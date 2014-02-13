//##########################################################################
//#                                                                        #
//#                            CLOUDCOMPARE                                #
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
//#          COPYRIGHT: EDF R&D / TELECOM ParisTech (ENST-TSI)             #
//#                                                                        #
//##########################################################################

#include "ccObject.h"

//Qt
#include <QSettings>

//System
#include <string.h>
#include <assert.h>
#include <stdint.h>

/** Versions:
   V1.0 = prior to 05/04/2012 = old version
   V2.0 - 05/04/2012 - upgrade to serialized version with version tracking
   V2.1 - 07/02/2012 - points & 2D labels upgraded
   V2.2 - 11/26/2012 - object name is now a QString
   V2.3 - 02/07/2013 - attribute 'm_selectionBehavior' added to ccHObject class
   v2.4 - 02/22/2013 - per-cloud point size + whether name is displayed in 3D or not
   v2.5 - 03/16/2013 - ccViewportParameters structure modified
   v2.6 - 04/03/2013 - strictly positive scalar field removed and 'hidden' values marker is now NaN
   v2.7 - 04/12/2013 - Customizable color scales
   v2.8 - 07/12/2013 - Poylines are now supported
   v2.9 - 08/14/2013 - ccMeshGroup removed, ccSubMesh added
   v3.0 - 08/30/2013 - QObject's meta data structure added
   v3.1 - 09/25/2013 - ccPolyline width added
   v3.2 - 10/11/2013 - ccFacet (2D polygons) are now supported
   v3.3 - 12/19/2013 - global scale information is now saved for point clouds
   v3.4 - 01/09/2014 - ccIndexedTransformation and ccIndexedTransformationBuffer added + CC_CLASS_ENUM is now coded on 64 bits
   v3.5 - 02/13/2014 - ccSensor class updated
**/
const unsigned c_currentDBVersion = 35; //3.5

unsigned ccObject::GetCurrentDBVersion()
{
	return c_currentDBVersion;
}

void ccObject::ResetUniqueIDCounter()
{
    QSettings settings;
    //settings.beginGroup("UniqueID");
	settings.setValue("UniqueID",(unsigned)0);
	//settings.endGroup();
}

unsigned ccObject::GetNextUniqueID()
{
    unsigned lastID = GetLastUniqueID();
	++lastID;
	UpdateLastUniqueID(lastID);

	return lastID;
}

unsigned ccObject::GetLastUniqueID()
{
    return QSettings().value("UniqueID", 0).toInt();
}

void ccObject::UpdateLastUniqueID(unsigned lastID)
{
    QSettings().setValue("UniqueID", lastID);
}

ccObject::ccObject(QString name)
{
    m_flags = CC_ENABLED;
    m_uniqueID = GetNextUniqueID();
    setName(name.isEmpty() ? "unnamed" : name);
}

QString ccObject::getName() const
{
    return m_name;
}

void ccObject::setName(const QString& name)
{
	m_name = name;
}

unsigned ccObject::getUniqueID() const
{
    return m_uniqueID;
}

void ccObject::setUniqueID(unsigned ID)
{
	m_uniqueID = ID;

	//updates last unique ID
	if (m_uniqueID>GetLastUniqueID())
		UpdateLastUniqueID(m_uniqueID);
}

bool ccObject::getFlagState(CC_OBJECT_FLAG flag) const
{
    return (m_flags & flag);
}

void ccObject::setFlagState(CC_OBJECT_FLAG flag, bool state)
{
    if (state)
        m_flags |= unsigned(flag);
    else
        m_flags &= (~unsigned(flag));
}

bool ccObject::isEnabled() const
{
    return getFlagState(CC_ENABLED);
}

void ccObject::setEnabled(bool state)
{
    setFlagState(CC_ENABLED,state);
}

bool ccObject::isLocked() const
{
    return getFlagState(CC_LOCKED);
}

void ccObject::setLocked(bool state)
{
    setFlagState(CC_LOCKED,state);
}

bool ccObject::toFile(QFile& out) const
{
	assert(out.isOpen() && (out.openMode() & QIODevice::WriteOnly));

	//class ID (dataVersion>=20)
	//DGM: on 64 bits since version 34
	uint64_t classID = static_cast<uint64_t>(getClassID());
	if (out.write((const char*)&classID,8)<0)
		return WriteError();

	//unique ID (dataVersion>=20)
	//DGM: this ID will be usefull to recreate dynamic links between entities!
	uint32_t uniqueID = (uint32_t)m_uniqueID;
	if (out.write((const char*)&uniqueID,4)<0)
		return WriteError();

	//name (dataVersion>=22)
	{
		QDataStream outStream(&out);
		outStream << m_name;
	}

	//flags (dataVersion>=20)
	uint32_t objFlags = (uint32_t)m_flags;
	if (out.write((const char*)&objFlags,4)<0)
		return WriteError();

	//meta data (dataVersion>=30)
	{
		//count
		uint32_t metaDataCount = (uint32_t)m_metaData.size();
		if (out.write((const char*)&metaDataCount,4)<0)
			return WriteError();

		//"key + value" pairs
		QDataStream outStream(&out);
		for (QVariantMap::const_iterator it = m_metaData.begin(); it != m_metaData.end(); ++it)
		{
			outStream << it.key();
			outStream << it.value();
		}
	}	

	return true;
}

CC_CLASS_ENUM ccObject::ReadClassIDFromFile(QFile& in, short dataVersion)
{
	assert(in.isOpen() && (in.openMode() & QIODevice::ReadOnly));

	//class ID (on 32 bits between version 2.0 and 3.3, then 64 bits from version 3.4)
	CC_CLASS_ENUM classID = CC_TYPES::OBJECT;
	if (dataVersion < 34)
	{
		uint32_t _classID = 0;
		if (in.read((char*)&_classID,4)<0)
			return ReadError();
		classID = static_cast<CC_CLASS_ENUM>(_classID);
	}
	else
	{
		uint64_t _classID = 0;
		if (in.read((char*)&_classID,8)<0)
			return ReadError();
		classID = static_cast<CC_CLASS_ENUM>(_classID);
	}
	
	return classID;
}

QVariant ccObject::getMetaData(QString key) const
{
	return m_metaData.value(key,QVariant());
}

bool ccObject::removeMetaData(QString key)
{
	return m_metaData.remove(key) != 0;
}

void ccObject::setMetaData(QString key, QVariant& data)
{
	m_metaData.insert(key,data);
}

bool ccObject::fromFile(QFile& in, short dataVersion, int flags)
{
	assert(in.isOpen() && (in.openMode() & QIODevice::ReadOnly));

	if (dataVersion<20)
		return CorruptError();

	//DGM: if we are here, we assume the class ID has already been read!
	//Call ccObject::readClassIDFromFile if necessary
	////class ID (dataVersion>=20)
	//uint32_t classID = 0;
	//if (in.read((char*)&classID,4)<0)
	//	return ReadError();

	//unique ID (dataVersion>=20)
	//DGM: this ID will be usefull to recreate dynamic links between entities!
	uint32_t uniqueID = 0;
	if (in.read((char*)&uniqueID,4)<0)
		return ReadError();
	m_uniqueID = (unsigned)uniqueID;

	//name
	if (dataVersion < 22) //old style
	{
		char name[256];
		if (in.read(name,256)<0)
			return ReadError();
		setName(name);
	}
	else //(dataVersion>=22)
	{
		QDataStream inStream(&in);
		inStream >> m_name;
	}

	//flags (dataVersion>=20)
	uint32_t objFlags = 0;
	if (in.read((char*)&objFlags,4)<0)
		return ReadError();
	m_flags = (unsigned)objFlags;

	//meta data (dataVersion>=30)
	if (dataVersion >= 30)
	{
		//count
		uint32_t metaDataCount = 0;
		if (in.read((char*)&metaDataCount,4)<0)
			return ReadError();

		//"key + value" pairs
		for (uint32_t i=0; i<metaDataCount; ++i)
		{
			QDataStream inStream(&in);
			QString key;
			QVariant value;
			inStream >> key;
			inStream >> value;
			setMetaData(key,value);
		}
	}	

	return true;
}
