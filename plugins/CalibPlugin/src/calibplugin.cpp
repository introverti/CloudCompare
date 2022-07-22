//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: CalibPlugin                      #
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
//#                             COPYRIGHT: XXX                             #
//#                                                                        #
//##########################################################################

// First:
//	Replace all occurrences of 'CalibPlugin' by your own plugin class name
// in this file. 	This includes the resource path to info.json in the
// constructor.

// Second:
//	Open CalibPlugin.qrc, change the "prefix" and the icon filename for
// your plugin. 	Change the name of the file to <yourPluginName>.qrc

// Third:
//	Open the info.json file and fill in the information about the plugin.
//	 "type" should be one of: "Standard", "GL", or "I/O" (required)
//	 "name" is the name of the plugin (required)
//	 "icon" is the Qt resource path to the plugin's icon (from the .qrc
// file) 	 "description" is used as a tootip if the plugin has actions and
// is displayed in the plugin dialog 	 "authors", "maintainers", and
// "references" show up in the plugin dialog as well

#include "calibplugin.h"

#include <ReferenceCloud.h>
#include <ccPointCloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <Eigen/Core>
#include <QMainWindow>
#include <QtGui>
#include <string>

#include "calibdlg.h"
#include "sm2cc.h"

QString parse_vector3f(const Eigen::Vector3f &vec) {
  std::string res = "[x,y,z] = [";
  res += std::to_string(vec(0));
  res += ",";
  res += std::to_string(vec(1));
  res += ",";
  res += std::to_string(vec(2));
  res += "]";
  return QString::fromStdString(res);
}
// Default constructor:
//	- pass the Qt resource path to the info.json file (from
//<yourPluginName>.qrc file)
//  - constructor should mainly be used to initialize actions and other members
CalibPlugin::CalibPlugin(QObject *parent)
    : QObject(parent),
      ccStdPluginInterface(":/CC/plugin/CalibPlugin/info.json"),
      m_action(nullptr),
      m_operator(std::make_shared<Calibration::Operator>()) {}

// This method should enable or disable your plugin actions
// depending on the currently selected entities ('selectedEntities').
void CalibPlugin::onNewSelection(const ccHObject::Container &selectedEntities) {
  if (m_action == nullptr) {
    return;
  }

  // If you need to check for a specific type of object, you can use the methods
  // in ccHObjectCaster.h or loop and check the objects' classIDs like this:
  //
  //	for ( ccHObject *object : selectedEntities )
  //	{
  //		if ( object->getClassID() == CC_TYPES::VIEWPORT_2D_OBJECT )
  //		{
  //			// ... do something with the viewports
  //		}
  //	}

  // For example - only enable our action if something is selected.
  m_action->setEnabled(!selectedEntities.empty());
}

// This method returns all the 'actions' your plugin can perform.
// getActions() will be called only once, when plugin is loaded.
QList<QAction *> CalibPlugin::getActions() {
  // default action (if it has not been already created, this is the moment to
  // do it)
  if (!m_action) {
    // Here we use the default plugin name, description, and icon,
    // but each action should have its own.
    m_action = new QAction(getName(), this);
    m_action->setToolTip(getDescription());
    m_action->setIcon(getIcon());

    // Connect appropriate signal
    connect(m_action, &QAction::triggered, this,
            [this]() { this->doAction(); });
  }

  return {m_action};
}

void CalibPlugin::doAction() {
  if (m_app == nullptr) {
    Q_ASSERT(false);
    return;
  }
  CalibDlg dlg(m_app->getMainWindow());
  if (!dlg.exec()) {
    return;
  }
  ccPointCloud *cloud;
  const ccHObject::Container &selectedEntities = m_app->getSelectedEntities();
  if (selectedEntities.size() < 1) {
    m_app->dispToConsole("[Calibration] Please select at least one pointcloud",
                         ccMainAppInterface::ERR_CONSOLE_MESSAGE);
    return;
  }
  ccHObject *entity = selectedEntities[0];
  if (entity->isA(CC_TYPES::POINT_CLOUD)) {
    cloud = static_cast<ccPointCloud *>(entity);
  } else {
    m_app->dispToConsole("[Calibration] not a CC_TYPES::POINT_CLOUD",
                         ccMainAppInterface::ERR_CONSOLE_MESSAGE);
    return;
  }

  QString defaultSFName("intensity");
  int sfIdx = -1;
  if (cloud->getNumberOfScalarFields() >= 1) {
    if (!defaultSFName.isEmpty()) {
      // if it's valid, we'll keep this SF!
      sfIdx = cloud->getScalarFieldIndexByName(qPrintable(defaultSFName));
    }
  }
  if (sfIdx >= 0) {
    CCCoreLib::ScalarField *sf = cloud->getScalarField(sfIdx);
    CCCoreLib::ReferenceCloud sampledCloud(cloud);

    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PCLPointCloud2::Ptr output_cloud2(new pcl::PCLPointCloud2);
    for (unsigned int i = 0; i < cloud->size(); ++i) {
      auto t_src = cloud->getPoint(i);
      pcl::PointXYZI t_point{t_src->x, t_src->y, t_src->z};
      t_point.intensity = float(sf->getValue(i));
      input_cloud->emplace_back(t_point);
    }
    Eigen::Vector3f feature_point;
    m_operator->setInputCloud(input_cloud);
    if (m_operator->compute(feature_point, output_cloud)) {
      m_app->dispToConsole(QString("[Calibration] Output Cloud Size %1")
                               .arg(output_cloud->size()),
                           ccMainAppInterface::STD_CONSOLE_MESSAGE);
      pcl::toPCLPointCloud2(*output_cloud, *output_cloud2);
      m_app->dispToConsole(QString("[Calibration] Output Cloud 2 Size %1")
                               .arg(output_cloud2->data.size()),
                           ccMainAppInterface::STD_CONSOLE_MESSAGE);
      auto cc_cloud_ptr = pcl2cc::Convert(*output_cloud2);
      m_app->dispToConsole(QString("[Calibration] Output Cloud CC Size %1")
                               .arg(cc_cloud_ptr->size()),
                           ccMainAppInterface::STD_CONSOLE_MESSAGE);
      if (cc_cloud_ptr) {
        cc_cloud_ptr->setName(cloud->getName() + QString(".Calibration"));
        cc_cloud_ptr->setGlobalShift(cloud->getGlobalShift());
        cc_cloud_ptr->setGlobalScale(cloud->getGlobalScale());
        cc_cloud_ptr->setDisplay(cloud->getDisplay());
        cc_cloud_ptr->prepareDisplayForRefresh();
        if (cloud->getParent()) cloud->getParent()->addChild(cc_cloud_ptr);
        cloud->setEnabled(false);
        m_app->addToDB(cc_cloud_ptr);
        cc_cloud_ptr->prepareDisplayForRefresh();
        // ccLog::Print(QString("[Picked] ") + m_label->getName());
        m_app->dispToConsole(
            QString("[Calibration] Get center point coordinate %1")
                .arg(parse_vector3f(feature_point)),
            ccMainAppInterface::STD_CONSOLE_MESSAGE);
      } else {
        m_app->dispToConsole(
            "[Calibration] Failed to convert between CC and PCL!",
            ccMainAppInterface::STD_CONSOLE_MESSAGE);
      }
    } else {
      m_app->dispToConsole("[Calibration] Failed to find intensity data!",
                           ccMainAppInterface::STD_CONSOLE_MESSAGE);
    }
  }
}
