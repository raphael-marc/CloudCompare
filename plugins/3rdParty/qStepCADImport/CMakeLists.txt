cmake_minimum_required( VERSION 3.0 )

option( PLUGIN_IO_QSTEP "Check to install the STEP reading file plugin" ON )

if ( PLUGIN_IO_QSTEP )
	project( QSTEP_IO_PLUGIN )

	set( CC_IS_IO_PLUGIN 1 )
	set( OPENCASCADE "/home/raphael/Logiciels/Salome_9.4/SALOME-9.4.0-UB18.04-SRC/BINARIES-UB18.04/CAS")
	include_directories("${OPENCASCADE}/include/opencascade")
    link_directories("${OPENCASCADE}/lib" CACHE PATH "OCC STEP lib")
	include(../../CMakePluginTpl.cmake )
	target_link_libraries(${PROJECT_NAME} TKSTEP TKSTEPBase TKSTEPAttr TKSTEP209 TKShHealing TKTopAlgo TKBRep TKGeomBase TKG3d TKG2d TKMath TKernel TKXSBase TKGeomAlgo TKMesh)

endif()

