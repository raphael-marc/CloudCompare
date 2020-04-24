#ifndef CC_STEP_FILTER_HEADER
#define CC_STEP_FILTER_HEADER

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

#include <FileIOFilter.h>

#include "DgmOctree.h"
using namespace CCLib;


class STEPFilter : public FileIOFilter
{
public:
	STEPFilter();
	
	// inherited from FileIOFilter
	CC_FILE_ERROR loadFile( const QString &fullFilename, ccHObject &container, LoadParameters &parameters ) override;

	CC_FILE_ERROR STEPImportFile(ccHObject &container, const QString &path, 
		const QString &file, float linearDeflection, LoadParameters &parameters);

};

#endif //CC_STEP_FILTER_HEADER
