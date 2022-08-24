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

#include <ccPointCloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <Eigen/Core>
#include <QMainWindow>
#include <QtGui>
#include <fstream>
#include <iostream>
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

    // If you need to check for a specific type of object, you can use the
    // methods in ccHObjectCaster.h or loop and check the objects' classIDs like
    // this:
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
    // null app
    if (m_app == nullptr) {
        Q_ASSERT(false);
        return;
    }
    // fail to launch qwidget
    CalibDlg dlg(m_app->getMainWindow());
    if (!dlg.exec()) {
        return;
    }
    // check selected element number
    const ccHObject::Container &selectedEntities = m_app->getSelectedEntities();
    // this case should never occur
    if (selectedEntities.size() < 1) {
        m_app->dispToConsole(
            "[Calibration] Please select at least one pointcloud",
            ccMainAppInterface::ERR_CONSOLE_MESSAGE);
        return;
    }
    // check the destination file address
    const QString qs_addr = dlg.openFilesPath;
    const std::string file_addr = qs_addr.toStdString();
    const QString defaultSFName = dlg.lidar_field_box->currentText();
    const QString lidar_model = dlg.lidar_type_box->currentText();
    if (lidar_model == "Jaguar") {
        m_operator->setLidarModel(1);
    } else if (lidar_model == "Falcon") {
        m_operator->setLidarModel(2);
    }
    std::fstream fd(file_addr, std::ios::out);
    if (!fd.is_open()) {
        m_app->dispToConsole(
            QString("[Calibration] %1 : Not a valid path.").arg(qs_addr),
            ccMainAppInterface::ERR_CONSOLE_MESSAGE);
        return;
    }
    size_t end_ite = file_addr.find_last_of('/');
    std::string debug_folder =
        end_ite > 0 ? file_addr.substr(0, end_ite) : "";
    // header
    fd << "[lidar]" << std::endl;
    // main process
    for (size_t i = 0; i < selectedEntities.size(); ++i) {
        ccPointCloud *cloud;
        ccHObject *entity = selectedEntities[i];
        // check entity type
        if (entity->isA(CC_TYPES::POINT_CLOUD)) {
            cloud = static_cast<ccPointCloud *>(entity);
            int sfIdx = -1;
            // check ScalarFields
            if (cloud->getNumberOfScalarFields() >= 1) {
                sfIdx =
                    cloud->getScalarFieldIndexByName(qPrintable(defaultSFName));
                // check ScalarFiled offset
                if (sfIdx >= 0) {
                    CCCoreLib::ScalarField *sf = cloud->getScalarField(sfIdx);
                    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(
                        new pcl::PointCloud<pcl::PointXYZI>);
                    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(
                        new pcl::PointCloud<pcl::PointXYZI>);
                    pcl::PCLPointCloud2::Ptr output_cloud2(
                        new pcl::PCLPointCloud2);
                    for (unsigned int ii = 0; ii < cloud->size(); ++ii) {
                        auto t_src = cloud->getPoint(ii);
                        pcl::PointXYZI t_point{t_src->x, t_src->y, t_src->z};
                        t_point.intensity = float(sf->getValue(ii));
                        input_cloud->emplace_back(t_point);
                    }
                    Eigen::Vector3f feature_point;
                    m_operator->setInputCloud(input_cloud);
                    m_operator->setFrameName(std::to_string(i));
                    m_operator->setDebugFolder(debug_folder);
                    int comput_status =
                        m_operator->compute(feature_point, output_cloud);
                    if (comput_status == 0) {
                        m_app->dispToConsole(
                            QString("[Calibration] Output Cloud Size %1")
                                .arg(output_cloud->size()),
                            ccMainAppInterface::STD_CONSOLE_MESSAGE);
                        pcl::toPCLPointCloud2(*output_cloud, *output_cloud2);
                        auto cc_cloud_ptr = pcl2cc::Convert(*output_cloud2);
                        if (cc_cloud_ptr) {
                            cc_cloud_ptr->setName(cloud->getName() +
                                                  QString(".Calibration"));
                            cc_cloud_ptr->setGlobalShift(
                                cloud->getGlobalShift());
                            cc_cloud_ptr->setGlobalScale(
                                cloud->getGlobalScale());
                            cc_cloud_ptr->setDisplay(cloud->getDisplay());
                            cc_cloud_ptr->prepareDisplayForRefresh();
                            if (cloud->getParent())
                                cloud->getParent()->addChild(cc_cloud_ptr);
                            cloud->setEnabled(false);
                            m_app->addToDB(cc_cloud_ptr);
                            cc_cloud_ptr->prepareDisplayForRefresh();
                            m_app->dispToConsole(
                                QString("[Calibration] Get center point "
                                        "coordinate %1")
                                    .arg(parse_vector3f(feature_point)),
                                ccMainAppInterface::STD_CONSOLE_MESSAGE);
                            fd << feature_point(0) << "," << feature_point(1)
                               << "," << feature_point(2);
                        } else {
                            m_app->dispToConsole(
                                QString("[Calibration] Entity[%1] ccPointCloud "
                                        "is nullptr.")
                                    .arg(i),
                                ccMainAppInterface::ERR_CONSOLE_MESSAGE);
                        }
                    } else {
                        m_app->dispToConsole(
                            QString("[Calibration] Entity[%1] Failed to find "
                                    "feature point.")
                                .arg(i),
                            ccMainAppInterface::ERR_CONSOLE_MESSAGE);
                        m_app->dispToConsole(
                            QString("[Calibration] ErrorCode :%1 .")
                                .arg(comput_status),
                            ccMainAppInterface::ERR_CONSOLE_MESSAGE);
                    }
                } else {
                    m_app->dispToConsole(
                        QString("[Calibration] Entity[%1] ScalarFiled offset "
                                "is not valid.")
                            .arg(i),
                        ccMainAppInterface::ERR_CONSOLE_MESSAGE);
                }
            } else {
                m_app->dispToConsole(
                    QString(
                        "[Calibration] Entity[%1] doesnt have ScalarFields.")
                        .arg(i),
                    ccMainAppInterface::ERR_CONSOLE_MESSAGE);
            }
        } else {
            m_app->dispToConsole(QString("[Calibration] Entity[%1] type is not "
                                         "CC_TYPES::POINT_CLOUD.")
                                     .arg(i),
                                 ccMainAppInterface::ERR_CONSOLE_MESSAGE);
        }
        fd << std::endl;
    }
    fd.close();
    m_app->dispToConsole(
        QString("[Calibration] Save feature point coordinate to :  %1")
            .arg(qs_addr),
        ccMainAppInterface::STD_CONSOLE_MESSAGE);
}