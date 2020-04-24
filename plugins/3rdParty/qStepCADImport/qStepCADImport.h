#ifndef CC_STEP_IMPORT_HEADER
#define CC_STEP_IMPORT_HEADER

//##########################################################################
//#                                                                        #
//#                              CLOUDCOMPARE                              #
//#                                                                        #
//#  This program is free software; you can redistribute it and/or modify  #
//#  it under the terms of the GNU General Public License as published by  #
//#  the Free Software Foundation; version 2 or later of the License.      #
//#                                                                        #
//#  This program is distributed in the hope that it will be useful,       #
//#  but WITHOUT ANY WARRANTY; without even the implied warranty of        #
//#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          #
//#  GNU General Public License for more details.                          #
//#                                                                        #
//#          COPYRIGHT: CloudCompare project                               #
//#                                                                        #
//##########################################################################

#include "ccIOPluginInterface.h"

class qStepCADImport : public QObject, public ccIOPluginInterface
{
	Q_OBJECT
	Q_INTERFACES( ccIOPluginInterface )
	
	Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.qStepCADImport" FILE "info.json")
	
public:
	explicit qStepCADImport( QObject *parent = nullptr );
	
protected:
	void registerCommands( ccCommandLineInterface *inCmdLine ) override;
	
	FilterList getFilters() override;
};

#endif //CC_STEP_IMPORT_HEADER