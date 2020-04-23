//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qRDBIO                      #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 of the License.               #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: RIEGL Laser Measurement Systems GmbH               #
//#                                                                        #
//##########################################################################

#include "STEPFilter.h"

//Qt
#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <QIcon>
#include <QComboBox>

//CClib
#include <ScalarField.h>
//#include <STLFilter.h>

//qCC_db
#include <ccPlane.h>
#include <ccPointCloud.h>
#include <ccProgressDialog.h>
#include <ccMesh.h>
#include <ccHObject.h>
#include <ccMaterial.h>
#include <ccMaterialSet.h>
#include <ccLog.h>
#include <ccScalarField.h>

// Include OpenCascade :
#include <STEPControl_Reader.hxx>
#include <Interface_Static.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <IFSelect_PrintCount.hxx>

#include "BRepTools.hxx"
#include "BRepBuilderAPI_MakePolygon.hxx"
#include "BRepBuilderAPI_MakeFace.hxx"
#include "BRepBuilderAPI_MakeSolid.hxx"
#include "BRep_Builder.hxx"
#include "Poly_Triangulation.hxx"

#include "TopoDS.hxx"
#include "TopoDS_Compound.hxx"
#include "TopoDS_Solid.hxx"
#include "TopoDS_Shell.hxx"
#include "TopAbs_ShapeEnum.hxx"

#include "TopTools.hxx"
#include "TColgp_Array1OfPnt.hxx"
#include "TopExp_Explorer.hxx"
#include "TopExp.hxx"
#include "BRepBuilderAPI_Sewing.hxx"

#include "TopTools_ListOfShape.hxx"
#include <NCollection_List.hxx>

#include "BRepMesh.hxx"
#include "BRepMesh_FastDiscret.hxx"
#include "BRepMesh_IncrementalMesh.hxx"

//System
#include <string>
#include <assert.h>
#include <sstream>
#include <array>
#include <map>
using namespace std;

// See https://www.opencascade.com/doc/occt-7.0.0/refman/html/_top_abs___shape_enum_8hxx.html#a67b8aa38656811eaee45f9df08499667
std::map<int, std::string> shapesTypes = 
	{
           {0, "TopAbs_COMPOUND"},
           {1, "TopAbs_COMPSOLID"},
           {2, "TopAbs_SOLID"},
           {3, "TopAbs_SHELL"},
           {4, "TopAbs_FACE"},
           {5, "TopAbs_EDGE"},
           {6, "TopAbs_VERTEX"},
           {7, "TopAbs_SHAPE"}
    };

STEPFilter::STEPFilter()
	: FileIOFilter( {
					"_STEP OpenCascade Filter",
					DEFAULT_PRIORITY,	//priority
					QStringList{ "step", "stp" },
					"step",
					QStringList{ "STEP CAD file (*.step)"},
					QStringList{ "STEP CAD file (*.stp)" },
					Import
					} )
{
}

CC_FILE_ERROR STEPFilter::loadFile( const QString &fullFilename, ccHObject &container, LoadParameters &parameters )
{
	//try to open the file
	QFileInfo fi(fullFilename);
	if(!QFileInfo::exists(fullFilename))
		return CC_FERR_UNKNOWN_FILE;
	QString path = fi.path();
	QString fileNameOnly = fi.fileName();
	// ccLog::Print(QString("path = '%1'").arg(path));
	// ccLog::Print(QString("fileNameOnly = '%1'").arg(fileNameOnly));

	STEPImportFile(container, path, fileNameOnly, 1.0, 1.0, parameters);
	return CC_FERR_NO_ERROR; //int STEPImportFile(int argc, char *argv[])
}


// const PointCoordinateType c_defaultSearchRadius = static_cast<PointCoordinateType>(sqrt(ZERO_TOLERANCE));
// static bool TagDuplicatedVertices(	const CCLib::DgmOctree::octreeCell& cell,
// 									void** additionalParameters,
// 									CCLib::NormalizedProgress* nProgress/*=0*/)
// {
// 	std::vector<int>* equivalentIndexes = static_cast<std::vector<int>*>(additionalParameters[0]);

// 	//we look for points very near to the others (only if not yet tagged!)

// 	//structure for nearest neighbors search
// 	CCLib::DgmOctree::NearestNeighboursSphericalSearchStruct nNSS;
// 	nNSS.level = cell.level;
// 	nNSS.prepare(c_defaultSearchRadius, cell.parentOctree->getCellSize(nNSS.level));
// 	cell.parentOctree->getCellPos(cell.truncatedCode, cell.level, nNSS.cellPos, true);
// 	cell.parentOctree->computeCellCenter(nNSS.cellPos, cell.level, nNSS.cellCenter);
// 	//*/

// 	unsigned n = cell.points->size(); //number of points in the current cell

// 	//we already know some of the neighbours: the points in the current cell!
// 	try
// 	{
// 		nNSS.pointsInNeighbourhood.resize(n);
// 	}
// 	catch (.../*const std::bad_alloc&*/) //out of memory
// 	{
// 		return false;
// 	}

// 	//init structure with cell points
// 	{
// 		CCLib::DgmOctree::NeighboursSet::iterator it = nNSS.pointsInNeighbourhood.begin();
// 		for (unsigned i = 0; i < n; ++i, ++it)
// 		{
// 			it->point = cell.points->getPointPersistentPtr(i);
// 			it->pointIndex = cell.points->getPointGlobalIndex(i);
// 		}
// 		nNSS.alreadyVisitedNeighbourhoodSize = 1;
// 	}

// 	//for each point in the cell
// 	for (unsigned i = 0; i < n; ++i)
// 	{
// 		int thisIndex = static_cast<int>(cell.points->getPointGlobalIndex(i));
// 		if (equivalentIndexes->at(thisIndex) < 0) //has no equivalent yet 
// 		{
// 			cell.points->getPoint(i, nNSS.queryPoint);

// 			//look for neighbors in a (very small) sphere
// 			//warning: there may be more points at the end of nNSS.pointsInNeighbourhood than the actual nearest neighbors (k)!
// 			unsigned k = cell.parentOctree->findNeighborsInASphereStartingFromCell(nNSS, c_defaultSearchRadius, false);

// 			//if there are some very close points
// 			if (k > 1)
// 			{
// 				for (unsigned j = 0; j < k; ++j)
// 				{
// 					//all the other points are equivalent to the query point
// 					const unsigned& otherIndex = nNSS.pointsInNeighbourhood[j].pointIndex;
// 					if (static_cast<int>(otherIndex) != thisIndex)
// 						equivalentIndexes->at(otherIndex) = thisIndex;
// 				}
// 			}

// 			//and the query point is always root
// 			equivalentIndexes->at(thisIndex) = thisIndex;
// 		}

// 		if (nProgress && !nProgress->oneStep())
// 		{
// 			return false;
// 		}
// 	}

// 	return true;
// }
//================================================================================
CC_FILE_ERROR STEPFilter::STEPImportFile(ccHObject &container, const QString &path, 
	const QString &file, float linearDeflection, float angularDeflection,
	LoadParameters &parameters)
//================================================================================
{
QString fullFileName = path + "/" + file;
// ccLog::Print("Fonction STEPImportFile...");
// ccLog::Print("Fichier :"+fullFileName);
string s = fullFileName.toStdString();
Standard_CString filename=(Standard_CString)s.c_str();
// cout << "====> filename : " << filename << endl;
QFileInfo fi(fullFileName);
QString basename = fi.baseName();

const Standard_Real aLinearDeflection = linearDeflection;
const Standard_Real anAngularDeflection = angularDeflection;

STEPControl_Reader aReader;
IFSelect_ReturnStatus aStatus = aReader.ReadFile(filename);
int ic = Interface_Static::IVal("xstep.cascade.unit");
string unit = Interface_Static::CVal("xstep.cascade.unit");
Interface_Static::SetCVal("xstep.cascade.unit", "M");
unit = Interface_Static::CVal("xstep.cascade.unit");
// // Root transfers
if (aStatus == IFSelect_ReturnStatus::IFSelect_RetDone)
	{
	bool isFailsonly = false;
	aReader.PrintCheckLoad(isFailsonly, IFSelect_PrintCount::IFSelect_ItemsByEntity);
	aReader.PrintCheckTransfer(isFailsonly, IFSelect_PrintCount::IFSelect_ItemsByEntity);

	//int aNbRoot = aReader.NbRootsForTransfer();
	//aReader.PrintCheckTransfer(isFailsonly, IFSelect_PrintCount::IFSelect_ItemsByEntity);
	//Standard_Integer nbr = aReader.NbRootsForTransfer();
	// aReader.PrintCheckTransfer (failsonly, IFSelect_ItemsByEntity);
	//for (Standard_Integer n = 1; n<= nbr; n++) {
	//    aReader.TransferRoot(n);
	//	}
	// // Collecting resulting entities
	Standard_Integer nbr = aReader.NbRootsForTransfer();
    //aReader.PrintCheckTransfer (failsonly, IFSelect_ItemsByEntity);
    for (Standard_Integer n = 1; n<= nbr; n++) {
        cout << "STEP: Transferring Root " << n << endl;
		aReader.TransferRoot(n);
    }
  	aReader.TransferRoots();
	Standard_Integer nbs = aReader.NbShapes();
	ccLog::Print("Number fo shapes = "+QString::number(nbs));
	TopoDS_Shape aShape;
	if (nbs == 0) {
		cout << "No shapes" << endl;
		ccLog::Print("No shapes found in the STEP file.");
	}
	else {
		aShape = aReader.OneShape();
//	    for (Standard_Integer i=1; i<=nbs; i++) {
		// ccLog::Print("Transferring shape "+QString::number(i)+" from the STEP file.");
		// aShape = aReader.Shape(i);
		// BRepTools::Dump(aShape,std::cout);
		string st = shapesTypes[aShape.ShapeType()];
		cout << "ShapeType : " << st << endl;
		ccLog::Print(QString("Shape type :%1").arg(st.c_str()));
		// BRepMesh_IncrementalMesh aMesh(aShape, aLinearDeflection, Standard_False, anAngularDeflection);
		BRepMesh_IncrementalMesh aMesh(aShape, 0.001, Standard_True);

		// On commence par compter le nombre de sommets et de triangles de la
		// tesselation pour réserver la mémoire des strctures d'accueil de CloudCompare.
		// Exploration shapes avec lib. C++ OpenCascade :
		// BRep_Builder builder;
		// TopoDS_Compound comp;
		// builder.MakeCompound(comp);
		
		//BRep_Tool bt;
		// NCollection_List<TopoDS_Face> l_faces;
		int i,j;
		int triCount=0; // Nombre de triangles de la tesselation
		int vertCount=0;
		TopExp_Explorer expFaces;
		cout << "expFaces.Init 1..." << endl;
		for (i=0,expFaces.Init(aShape, TopAbs_FACE); expFaces.More(); i++,expFaces.Next()) {
		//for (i = 0 ; expFaces.More(); expFaces.Next(), i++) {
			const TopoDS_Face& face = TopoDS::Face(expFaces.Current());
			TopLoc_Location location;
			cout << i << " : avant bt.Triangulation..." << endl;
			
			// builder.Add(comp, face());
			// l_faces.Append(face());
			Poly_Triangulation facing = BRep_Tool::Triangulation(face, location);
			// Handle(Poly_Triangulation) facing = BRep_Tool::Triangulation(face, location);
			// if (facing.IsNull())
			// {
			// 	BRepMesh_FastDiscret myMesh(face);
			// 	facing = BRep_Tool::Triangulation(face, location);
			// }
			// if (facing.IsNull())
			// 	continue;
			cout << i << " : après bt.Triangulation..." << endl;
			gp_Trsf nodeTransformation = location;
			vertCount += facing.NbNodes();
			triCount += facing.NbTriangles();
		}
		ccLog::Print("Number of CAD faces  = "+QString::number(i));
		ccLog::Print("Number of triangles (after tesselation) = "+QString::number(triCount));
		ccLog::Print("Number of vertices (after tesselation)  = "+QString::number(vertCount));

		// Création des nuages de points et maillage dans CloudCompare :
		QString name("mesh from STEP file");
		ccPointCloud* vertices = new ccPointCloud("vertices");
		ccMesh* mesh = new ccMesh(vertices);
		mesh->setName(name);
		vertices->reserve(vertCount + 100);
		mesh->reserve(triCount + 100);
		
		unsigned pointCount = 0;
		expFaces.ReInit();
		cout << "expFaces.Init 2..." << endl;
		for (i=0,expFaces.Init(aShape, TopAbs_FACE); expFaces.More(); i++,expFaces.Next()) {
		//for (i = 1 ; expFaces.More(); expFaces.Next(), i++) {
			const TopoDS_Face& face = TopoDS::Face(expFaces.Current());
			TopLoc_Location location;
			Poly_Triangulation facing = BRep_Tool::Triangulation(face, location);

			gp_Trsf nodeTransformation = location;
			TColgp_Array1OfPnt nodes = facing.Nodes();
			Poly_Array1OfTriangle tri = facing.Triangles();
			//cout << "i=" << i << "  tri=" << facing.NbTriangles() << endl;
			for (j=1; j<=facing.NbTriangles();j++) {
				//cout << "\t\tj=" << j << endl;
				Poly_Triangle trian = tri.Value(j);
				Standard_Integer index1, index2, index3;
				trian.Get(index1, index2, index3);
				const gp_Pnt& p1 = nodes.Value(index1).Transformed(nodeTransformation);
				const gp_Pnt& p2 = nodes.Value(index2).Transformed(nodeTransformation);
				const gp_Pnt& p3 = nodes.Value(index3).Transformed(nodeTransformation);

		//       BRepBuilderAPI_MakePolygon poly(BRepBuilderAPI_MakePolygon
		//       									(p1,p2,p3,true));
		//       poly.Close();
		//       BRepBuilderAPI_MakeFace f(poly.Wire());
				// if (f.IsDone()) {
		//       	builder.Add(comp, f.Face());
				//     l_faces.Append(f.Face());
		//       }
				// Récupération des coordonnées des sommets du triangle courant :
				unsigned vertIndexes[3];
				vertIndexes[0] = pointCount++;
				CCVector3 P(p1.X(),p1.Y(),p1.Z());vertices->addPoint(P);
				vertIndexes[1] = pointCount++;
				P.x = p2.X();P.y = p2.Y();P.z = p2.Z();vertices->addPoint(P);
				vertIndexes[2] = pointCount++;
				P.x = p3.X();P.y = p3.Y();P.z = p3.Z();vertices->addPoint(P);
				mesh->addTriangle(vertIndexes[0], vertIndexes[1], vertIndexes[2]);
				//mesh->addTriangle(index1,index2,index3);
			}
		}
//===============================================================================
//do some cleaning
//===============================================================================
	// ccLog::Print("Nombre de faces = "+QString::number(i));

		vertices->setEnabled(true);
		vertices->setLocked(false); //DGM: no need to lock it as it is only used by one mesh!
		mesh->addChild(vertices);
		container.addChild(mesh);

		// BRepBuilderAPI_Sewing sewing;
		// for(const TopoDS_Shape& face : l_faces) {
		//     sewing.Add(face);
		// }
		// sewing.Perform();
		// TopoDS_Shell sewed_shape = (TopoDS_Shell&)sewing.SewedShape();

		// QString f = path + "/output/export_"+basename+".brep";
		// s = f.toStdString();
		// Standard_CString export_filename=(Standard_CString)s.c_str();
		// if (BRepTools::Write(sewed_shape, export_filename))
		// 	ccLog::Print("File 'export_"+basename+".brep'"+" exported successfully");
		}
	}
}