//##########################################################################
//#                                                                        #
//#                CLOUDCOMPARE PLUGIN: EvaluatePlugin                      #
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
//	Replace all occurrences of 'EvaluatePlugin' by your own plugin class
// name
// in this file. 	This includes the resource path to info.json in the
// constructor.

// Second:
//	Open EvaluatePlugin.qrc, change the "prefix" and the icon filename for
// your plugin. 	Change the name of the file to <yourPluginName>.qrc

// Third:
//	Open the info.json file and fill in the information about the plugin.
//	 "type" should be one of: "Standard", "GL", or "I/O" (required)
//	 "name" is the name of the plugin (required)
//	 "icon" is the Qt resource path to the plugin's icon (from the .qrc
// file) 	 "description" is used as a tootip if the plugin has actions and
// is displayed in the plugin dialog 	 "authors", "maintainers", and
// "references" show up in the plugin dialog as well

#include "evaluateplugin.h"

#include <ccPointCloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <Eigen/Core>
#include <QMainWindow>
#include <QtGui>
#include <fstream>
#include <iostream>
#include <string>

#include "evaluatedlg.h"
#include "sm2cc.h"

ccPointCloud *pcl_to_cc(pcl::PointCloud<pcl::PointXYZI>::Ptr source) {
  pcl::PCLPointCloud2::Ptr source_pcl2(new pcl::PCLPointCloud2);
  pcl::toPCLPointCloud2(*source, *source_pcl2);
  ccPointCloud *cc_cloud_ptr = pcl2cc::Convert(*source_pcl2);
  return cc_cloud_ptr;
}

EvaluatePlugin::EvaluatePlugin(QObject *parent)
    : QObject(parent),
      ccStdPluginInterface(":/CC/plugin/EvaluatePlugin/info.json"),
      m_action(nullptr),
      m_operator(Evaluation::Evaluation()) {}

void EvaluatePlugin::onNewSelection(
    const ccHObject::Container &selectedEntities) {
  if (m_action == nullptr) {
    return;
  }
  m_action->setEnabled(!selectedEntities.empty());
}

QList<QAction *> EvaluatePlugin::getActions() {
  if (!m_action) {
    m_action = new QAction(getName(), this);
    m_action->setToolTip(getDescription());
    m_action->setIcon(getIcon());
    connect(m_action, &QAction::triggered, this,
            [this]() { this->doAction(); });
  }

  return {m_action};
}

void EvaluatePlugin::doAction() {
  // null app
  if (m_app == nullptr) {
    Q_ASSERT(false);
    return;
  }
  // check selected element number
  const ccHObject::Container &selectedEntities = m_app->getSelectedEntities();
  // this case should never occur
  if (selectedEntities.size() < 1) {
    m_app->dispToConsole("[Calibration] Please select at least one pointcloud",
                         ccMainAppInterface::ERR_CONSOLE_MESSAGE);
    return;
  }
  // fail to launch qwidget
  EvaluateDlg dlg(m_app->getMainWindow());
  if (!dlg.exec()) {
    return;
  }
  int param_iter = dlg.NIteration->value();
  double param_threshold = dlg.Nthreshold->value();
  double param_tolerence = dlg.Ntolerence->value();
  ccPointCloud *cloud;
  ccHObject *entity = selectedEntities[0];
  if (!entity->isA(CC_TYPES::POINT_CLOUD)) {
    return;
  }
  cloud = static_cast<ccPointCloud *>(entity);
  if (cloud->getNumberOfScalarFields() < 1) {
    return;
  }
  int ref_id = -1;
  int src_id = -1;
  QString reflectance = "reflectance";
  QString intensity = "intensity";
  QString origin_source = "Original_cloud_index";
  ref_id = cloud->getScalarFieldIndexByName(qPrintable(reflectance));
  src_id = cloud->getScalarFieldIndexByName(qPrintable(origin_source));

  if (ref_id < 0) {
    ref_id = cloud->getScalarFieldIndexByName(qPrintable(intensity));
    if (ref_id < 0) {
      m_app->dispToConsole("[Evaluation] Failed to find reflectence",
                           ccMainAppInterface::ERR_CONSOLE_MESSAGE);
      return;
    }
  }
  if (src_id < 0) {
    m_app->dispToConsole("[Evaluation] Failed to find Original_cloud_index",
                         ccMainAppInterface::ERR_CONSOLE_MESSAGE);
  }
  CCCoreLib::ScalarField *ref_field = cloud->getScalarField(ref_id);
  CCCoreLib::ScalarField *src_field = cloud->getScalarField(src_id);

  pcl::PointCloud<pcl::PointXYZI>::Ptr first_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr second_cloud(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr first_res(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr second_res(
      new pcl::PointCloud<pcl::PointXYZI>);

  for (unsigned int i = 0; i < cloud->size(); ++i) {
    auto t_src = cloud->getPoint(i);
    pcl::PointXYZI t_point{t_src->x, t_src->y, t_src->z};
    t_point.intensity = float(ref_field->getValue(i));
    if (src_field->getValue(i) == 0) {
      first_cloud->emplace_back(t_point);
    } else if (src_field->getValue(i) == 1) {
      second_cloud->emplace_back(t_point);
    }
  }
  m_operator.set_ground_param(param_iter, param_threshold, param_tolerence);
  m_operator.compute(first_cloud, second_cloud, first_res, second_res);
  auto first_cc_ptr = pcl_to_cc(first_res);
  auto second_cc_ptr = pcl_to_cc(second_res);

  first_cc_ptr->setName(cloud->getName() + QString(".first"));
  first_cc_ptr->setGlobalShift(cloud->getGlobalShift());
  first_cc_ptr->setGlobalScale(cloud->getGlobalScale());
  first_cc_ptr->setDisplay(cloud->getDisplay());
  first_cc_ptr->prepareDisplayForRefresh();
  if (cloud->getParent()) cloud->getParent()->addChild(first_cc_ptr);
  cloud->setEnabled(false);
  m_app->addToDB(first_cc_ptr);
  first_cc_ptr->prepareDisplayForRefresh();

  second_cc_ptr->setName(cloud->getName() + QString(".second"));
  second_cc_ptr->setGlobalShift(cloud->getGlobalShift());
  second_cc_ptr->setGlobalScale(cloud->getGlobalScale());
  second_cc_ptr->setDisplay(cloud->getDisplay());
  second_cc_ptr->prepareDisplayForRefresh();
  if (cloud->getParent()) cloud->getParent()->addChild(second_cc_ptr);
  cloud->setEnabled(false);
  m_app->addToDB(second_cc_ptr);
  second_cc_ptr->prepareDisplayForRefresh();
}