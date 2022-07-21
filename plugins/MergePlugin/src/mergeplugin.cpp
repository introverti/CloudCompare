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

#include "mergeplugin.h"

#include <ccObject.h>
#include <ccPointCloud.h>

#include <QMainWindow>
#include <QtGui>

#include "mergedlg.h"

MergePlugin::MergePlugin(QObject *parent)
    : QObject(parent),
      ccStdPluginInterface(":/CC/plugin/MergePlugin/info.json"),
      m_action(nullptr) {}

void MergePlugin::onNewSelection(const ccHObject::Container &selectedEntities) {
  if (m_action == nullptr) {
    return;
  }
  m_action->setEnabled(!selectedEntities.empty());
}

QList<QAction *> MergePlugin::getActions() {
  if (!m_action) {
    m_action = new QAction(getName(), this);
    m_action->setToolTip(getDescription());
    m_action->setIcon(getIcon());
    connect(m_action, &QAction::triggered, this,
            [this]() { this->doAction(); });
  }

  return {m_action};
}

void MergePlugin::doAction() {
  if (m_app == nullptr) {
    Q_ASSERT(false);
    return;
  }
  MergeDlg dlg(m_app->getMainWindow());
  if (!dlg.exec()) {
    return;
  }
  QString p_name = dlg.Pname->text();
  unsigned p_number = dlg.Pnumber->value();

  ccPointCloud *cloud;
  ccPointCloud *combined_cloud;
  const ccHObject::Container &selectedEntities = m_app->getSelectedEntities();
  if (selectedEntities.size() < 1) {
    m_app->dispToConsole("[Merge] please select at least one pointcloud",
                         ccMainAppInterface::ERR_CONSOLE_MESSAGE);
    return;
  }
  bool inited = false;
  for (unsigned int i = 0; i < selectedEntities.size(); ++i) {
    ccHObject *entity = selectedEntities[i];
    if (entity->isA(CC_TYPES::POINT_CLOUD)) {
      ccPointCloud *curr_ptr = static_cast<ccPointCloud *>(entity);
      if (inited) {
        cloud = curr_ptr->cloneThis(nullptr, true);
        combined_cloud->operator+=(cloud);
      } else {
        combined_cloud = curr_ptr->cloneThis();
        inited = true;
      }
    } else {
      m_app->dispToConsole("[Merge] not a CC_TYPES::POINT_CLOUD",
                           ccMainAppInterface::ERR_CONSOLE_MESSAGE);
      continue;
    }
    // m_app->removeFromDB(entity);
  }
  if (combined_cloud) {
    QString root_name = QString(p_name + "-%1").arg(p_number);
    ccHObject *root_group = new ccHObject(root_name, p_number);
    m_app->addToDB(root_group);
    combined_cloud->setName(root_name);
    combined_cloud->prepareDisplayForRefresh();
    root_group->addChild(combined_cloud);
    m_app->addToDB(combined_cloud);
    combined_cloud->prepareDisplayForRefresh();
    m_app->dispToConsole(QString("[Merge] Get merged pointcloud size %1")
                             .arg(combined_cloud->size()),
                         ccMainAppInterface::STD_CONSOLE_MESSAGE);
  } else {
    m_app->dispToConsole("[Merge] No valid CC_TYPES::POINT_CLOUD",
                         ccMainAppInterface::STD_CONSOLE_MESSAGE);
  }
}
