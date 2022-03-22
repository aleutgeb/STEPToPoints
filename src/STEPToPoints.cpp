//
// Program: STEPToPoints
//
// Description:
//
// The program STEPToPoints converts solids contained in STEP files into point clouds by regular sampling.
//
// Copyright(C) 2022 Alexander Leutgeb
//
// This library is free software; you can redistribute it and / or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110 - 1301  USA
//

#include <TopoDS_Solid.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Builder.hxx>
#include <TopoDS.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <TDocStd_Document.hxx>
#include <TDF_ChildIterator.hxx>
#include <TDF_LabelSequence.hxx>
#include <TDataStd_Name.hxx>
#include <IntCurvesFace_ShapeIntersector.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <gp_Lin.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <GeomLProp_SLProps.hxx>
#include <vector>
#include <set>
#include <array>
#include <numeric>
#include <cmath>
#include <execution>
#include <thread>
#include <iostream>
#include "cxxopts.hpp"

struct NamedSolid {
	NamedSolid(const TopoDS_Solid& s, const std::string& n) : solid{s}, name{n} {}

	const TopoDS_Solid solid;
	const std::string  name;
};

void getNamedSolids(const TopLoc_Location& location, const std::string& prefix, unsigned int& id, const Handle(XCAFDoc_ShapeTool) shapeTool,
		const TDF_Label label, std::vector<NamedSolid>& namedSolids) {
	TDF_Label referredLabel{label};
	if (shapeTool->IsReference(label)) shapeTool->GetReferredShape(label, referredLabel);
	std::string name;
	Handle(TDataStd_Name) shapeName;
	if (referredLabel.FindAttribute(TDataStd_Name::GetID(), shapeName)) name = TCollection_AsciiString(shapeName->Get()).ToCString();
	if (name == "") name = std::to_string(id++);
	std::string fullName{prefix + "/" + name};

	TopLoc_Location localLocation = location * shapeTool->GetLocation(label);
	TDF_LabelSequence components;
	if (shapeTool->GetComponents(referredLabel, components)) {
		for (Standard_Integer compIndex{1}; compIndex <= components.Length(); ++compIndex) {
			getNamedSolids(localLocation, fullName, id, shapeTool, components.Value(compIndex), namedSolids);
		}
	}
	else {
		TopoDS_Shape shape;
		shapeTool->GetShape(referredLabel, shape);
		if (shape.ShapeType() == TopAbs_SOLID) {
			BRepBuilderAPI_Transform transform(shape, localLocation, Standard_True);
			namedSolids.emplace_back(TopoDS::Solid(transform.Shape()), fullName);
		}
	}
}

void read(const std::string& inFile, std::vector<NamedSolid>& namedSolids) {
	Handle(TDocStd_Document) document;
	Handle(XCAFApp_Application) application = XCAFApp_Application::GetApplication();
	application->NewDocument(inFile.c_str(), document);
	STEPCAFControl_Reader reader;
	reader.SetNameMode(true);
	IFSelect_ReturnStatus stat{reader.ReadFile(inFile.c_str())};
	if (stat != IFSelect_RetDone || !reader.Transfer(document)) throw std::logic_error{std::string{"Could not read '"} + inFile + "'"};
	Handle(XCAFDoc_ShapeTool) shapeTool{XCAFDoc_DocumentTool::ShapeTool(document->Main())};
	TDF_LabelSequence topLevelShapes;
	shapeTool->GetFreeShapes(topLevelShapes);
	unsigned int id{1};
	for (Standard_Integer iLabel{1}; iLabel <= topLevelShapes.Length(); ++iLabel) {
		getNamedSolids(TopLoc_Location{}, "", id, shapeTool, topLevelShapes.Value(iLabel), namedSolids);
	}
}

struct Point {
	Point(const std::array<double, 3>& v, const std::array<double, 3>& n) : vertex{v}, normal{n} {}

	std::array<double, 3> vertex;
	std::array<double, 3> normal;
};

void writeXYZ(const std::string& outFile, const std::vector<Point>& points) {
	std::ofstream ofs{outFile};
	for (const auto& p : points) {
		ofs << p.vertex[0] << " " << p.vertex[1] << " " << p.vertex[2] << " " << p.normal[0] << " " << p.normal[1] << " " << p.normal[2] << "\n";
	}
	ofs.close();
}

struct PointLessOperator {
	PointLessOperator(const double e) : eps{e} {}

	bool operator()(const Point& lhs, const Point& rhs) const {
		for (auto i{0}; i < 3; ++i) {
			if (std::abs(lhs.vertex[i] - rhs.vertex[i]) > eps) return lhs.vertex[i] < rhs.vertex[i];
		}
		return false;
	}

	const double eps;
};

auto makeUniquePoints(const std::vector<Point>& points, const double epsilon) {
	std::vector<Point> result;

	PointLessOperator lessOperator{epsilon};
	std::set<Point, decltype(lessOperator)> pointSet{lessOperator};
	for (const auto& p : points) pointSet.insert(p);
	for (const auto& key : pointSet) result.emplace_back(key);
	return result;
}

auto createScanLines(const TopoDS_Shape& shape, const double sampling) -> std::vector<gp_Lin> {
	std::vector<gp_Lin> result;
	std::array<double, 3> min;
	std::array<double, 3> max;
	Bnd_Box box;
	BRepBndLib::Add(shape, box);
	box.Get(min[0], min[1], min[2], max[0], max[1], max[2]);
	for (auto dim{0u}; dim < 3u; ++dim) {
		const double uMin{min[(dim + 1) % 3]};
		const double uMax{max[(dim + 1) % 3]};
		const double vMin{min[(dim + 2) % 3]};
		const double vMax{max[(dim + 2) % 3]};
		std::array<double, 3> direction{0.0, 0.0, 0.0};
		direction[dim] = 1.0;
		std::array<double, 3> point{min};
		for (double u{uMin}; u < uMax; u += sampling) {
			for (double v{vMin}; v < vMax; v += sampling) {
				point[(dim + 1) % 3] = u;
				point[(dim + 2) % 3] = v;
				result.emplace_back(gp_Pnt{point[0], point[1], point[2]}, gp_Dir{direction[0], direction[1], direction[2]});
			}
		}
	}
	return result;
}

auto surfaceNormal(const TopoDS_Face& face, const double u, const double v, const double resolution) -> gp_Dir {
	Handle(Geom_Surface) surface{BRep_Tool::Surface(face)};
	GeomLProp_SLProps props{surface, u, v, 1, resolution};
	gp_Dir normal{props.Normal()};
	if (face.Orientation() == TopAbs_REVERSED) normal.Reverse();
	return normal;
}

auto sampleShape(const TopoDS_Shape& shape, const double sampling) -> std::vector<Point> {
	std::vector<Point> result;
	std::vector<gp_Lin> scanLines{createScanLines(shape, sampling)};
	const double tolerance{sampling * 0.001};
	const auto numThreads{std::thread::hardware_concurrency()};
	std::vector<IntCurvesFace_ShapeIntersector> tlsIntersectors(numThreads);
	for (auto& intersector : tlsIntersectors) intersector.Load(shape, tolerance);
	std::vector<std::vector<Point>> tlsResult(numThreads);
	std::vector<std::vector<gp_Lin>> tlsScanLines(numThreads);
	for (std::size_t i{0}; i < scanLines.size(); ++i) tlsScanLines[i % numThreads].emplace_back(scanLines[i]);
	std::vector<int> threadIDs(numThreads);
	std::iota(std::begin(threadIDs), std::end(threadIDs), 0);
	std::for_each(std::execution::par, std::begin(threadIDs), std::end(threadIDs), [&](int threadID) {
		for (const auto& scanLine : tlsScanLines[threadID]) {
			auto& intersector{tlsIntersectors[threadID]};
			intersector.Perform(scanLine, -RealLast(), RealLast());
			if (!intersector.IsDone()) continue;
			for (auto i{1}; i <= intersector.NbPnt(); ++i) {
				const gp_Pnt p{intersector.Pnt(i)};
				const TopoDS_Face f{intersector.Face(i)};
				const gp_Dir n{surfaceNormal(f, intersector.UParameter(i), intersector.VParameter(i), tolerance)};
				tlsResult[threadID].emplace_back(std::array<double, 3>{p.X(), p.Y(), p.Z()}, std::array<double, 3>{n.X(), n.Y(), n.Z()});
			}
		}
	});
	for (const auto& r : tlsResult) std::copy(std::begin(r), std::end(r), std::back_inserter(result));
	return result;
}

void write(const std::string& outFile, const std::vector<NamedSolid>& namedSolids, const std::vector<std::string>& select, const double sampling) {
	TopoDS_Compound compound;
	TopoDS_Builder builder;
	builder.MakeCompound(compound);
	if (select.empty()) {
		for (const auto& namedSolid : namedSolids) builder.Add(compound, namedSolid.solid);
	}
	else {
		for (const auto& sel : select) {
			if (sel != "") {
				if (sel[0] == '/') {
					const auto iter{std::find_if(std::begin(namedSolids), std::end(namedSolids), [&](const auto& namesSolid) { return namesSolid.name == sel; })};
					if (iter == std::end(namedSolids)) throw std::logic_error{ std::string{"Could not find solid with name '"} + sel + "'" };
					builder.Add(compound, iter->solid);
				}
				else {
					try {
						int index{std::stoi(sel)};
						if (index < 1 || index > namedSolids.size()) throw std::logic_error{std::string{"Index out of range: "} + sel};
						builder.Add(compound, namedSolids[index - 1].solid);
					}
					catch (const std::invalid_argument&) {
						throw std::logic_error{std::string("Invalid index: ") + sel};
					}
				}
			}
		}
	}
	std::vector<Point> points{makeUniquePoints(sampleShape(compound, sampling), sampling * 0.001)};
	writeXYZ(outFile, points);
}

int main(int argc, char* argv[]) {
	cxxopts::Options options{"STEPToPoints", "STEP to point cloud conversion by regular sampling"};
	options.add_options()
			("i,in", "Input file", cxxopts::value<std::string>())
			("o,out", "Output file", cxxopts::value<std::string>())
			("c,content", "List content (solids)")
			("s,select", "Select solids by name or index (comma seperated list, index starts with 1)", cxxopts::value<std::vector<std::string>>())
			("g,sampling", "Sampling distance", cxxopts::value<double>())
			("h,help", "Print usage");
	try {
		const auto result{options.parse(argc, argv)};
		if (result.count("content")) {
			if (result.count("in")) {
				const std::string inFile{result["in"].as<std::string>()};
				std::vector<NamedSolid> namedSolids;
				read(inFile, namedSolids);
				for (const auto& namedSolid : namedSolids) std::cout << namedSolid.name << std::endl;
			}
			else throw std::logic_error{std::string{"Missing option 'in'"}};
		}
		else if (result.count("in") && result.count("out")) {
			const auto inFile{result["in"].as<std::string>()}, outFile{result["out"].as<std::string>()};
			if (!result.count("sampling")) throw std::logic_error{std::string{"Missing option 'sampling'"}};
			const auto sampling{result["sampling"].as<double>()};
			std::vector<std::string> select;
			if (result.count("select")) select = result["select"].as<std::vector<std::string>>();
			std::vector<NamedSolid> namedSolids;
			read(inFile, namedSolids);
			write(outFile, namedSolids, select, sampling);
		}
		else std::cout << options.help() << std::endl;
		return EXIT_SUCCESS;
	}
	catch (const std::exception& ex) {
		std::cerr << ex.what();
		return EXIT_FAILURE;
	}
	catch (...) {
		std::cerr << "Unexpected exception";
		return EXIT_FAILURE;
	}
}
