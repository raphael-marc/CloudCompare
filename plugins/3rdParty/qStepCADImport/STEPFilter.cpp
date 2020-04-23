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
//================================================================================
CC_FILE_ERROR STEPFilter::loadFile( const QString &fullFilename, ccHObject &container, 
									LoadParameters &parameters )
//================================================================================
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
//================================================================================
CC_FILE_ERROR STEPFilter::STEPImportFile(ccHObject &container, const QString &path, 
	const QString &file, float linearDeflection, float angularDeflection,
	LoadParameters &parameters)
//================================================================================
{
QString fullFileName = path + "/" + file;
string s = fullFileName.toStdString();
Standard_CString filename=(Standard_CString)s.c_str();
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
	// Collecting resulting entities
	Standard_Integer nbr = aReader.NbRootsForTransfer();
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
		string st = shapesTypes[aShape.ShapeType()];
		cout << "ShapeType : " << st << endl;
		ccLog::Print(QString("Shape type :%1").arg(st.c_str()));
		BRepMesh_IncrementalMesh aMesh(aShape, 0.001, Standard_True);

		// On commence par compter le nombre de sommets et de triangles de la
		// tesselation pour réserver la mémoire des strctures d'accueil de CloudCompare.
		// Exploration shapes avec lib. C++ OpenCascade :
		int i,j;
		int triCount=0; // Nombre de triangles de la tesselation
		int vertCount=0;
		TopExp_Explorer expFaces;
		cout << "expFaces.Init 1..." << endl;
		for (i=0,expFaces.Init(aShape, TopAbs_FACE); expFaces.More(); i++,expFaces.Next()) {
			const TopoDS_Face& face = TopoDS::Face(expFaces.Current());
			TopLoc_Location location;
			Poly_Triangulation facing = BRep_Tool::Triangulation(face, location);
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
			const TopoDS_Face& face = TopoDS::Face(expFaces.Current());
			TopLoc_Location location;
			Poly_Triangulation facing = BRep_Tool::Triangulation(face, location);

			gp_Trsf nodeTransformation = location;
			TColgp_Array1OfPnt nodes = facing.Nodes();
			Poly_Array1OfTriangle tri = facing.Triangles();
			for (j=1; j<=facing.NbTriangles();j++) {
				Poly_Triangle trian = tri.Value(j);
				Standard_Integer index1, index2, index3;
				trian.Get(index1, index2, index3);
				const gp_Pnt& p1 = nodes.Value(index1).Transformed(nodeTransformation);
				const gp_Pnt& p2 = nodes.Value(index2).Transformed(nodeTransformation);
				const gp_Pnt& p3 = nodes.Value(index3).Transformed(nodeTransformation);
				// Récupération des coordonnées des sommets du triangle courant :
				unsigned vertIndexes[3];
				vertIndexes[0] = pointCount++;
				CCVector3 P(p1.X(),p1.Y(),p1.Z());vertices->addPoint(P);
				vertIndexes[1] = pointCount++;
				P.x = p2.X();P.y = p2.Y();P.z = p2.Z();vertices->addPoint(P);
				vertIndexes[2] = pointCount++;
				P.x = p3.X();P.y = p3.Y();P.z = p3.Z();vertices->addPoint(P);
				mesh->addTriangle(vertIndexes[0], vertIndexes[1], vertIndexes[2]);
			}
		}
//===============================================================================
// ==> DGM : do some cleaning, fuse the vertices that are duplicated
//===============================================================================
		vertices->setEnabled(true);
		vertices->setLocked(false);
		mesh->addChild(vertices);
		container.addChild(mesh);
		}
	}
}