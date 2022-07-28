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

#pragma once
#include "ccStdPluginInterface.h"
#include "evaluation.h"

class EvaluatePlugin : public QObject, public ccStdPluginInterface {
  Q_OBJECT
  Q_INTERFACES(ccPluginInterface ccStdPluginInterface)
  Q_PLUGIN_METADATA(IID "cccorp.cloudcompare.plugin.EvaluatePlugin" FILE
                        "../info.json")

 public:
  explicit EvaluatePlugin(QObject *parent = nullptr);
  ~EvaluatePlugin() override = default;
  void onNewSelection(const ccHObject::Container &selectedEntities) override;
  QList<QAction *> getActions() override;
  void doAction();

 private:
  QAction *m_action;
  Evaluation::Evaluation m_operator;
};