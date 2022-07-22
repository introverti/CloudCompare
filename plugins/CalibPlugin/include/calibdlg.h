/***
 * @Copyright [2022] <Innovusion Inc.>
 * @LastEditTime: 2022-07-22 15:22:23
 * @LastEditors: Tianyun Xuan
 */
/***
 * @Copyright [2022] <Innovusion Inc.>
 * @LastEditTime: 2022-07-21 14:47:22
 * @LastEditors: Tianyun Xuan
 */
//##########################################################################
//#                                                                        #
//#                       CLOUDCOMPARE PLUGIN: qPCV                        #
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
//#                  COPYRIGHT: Daniel Girardeau-Montaut                   #
//#                                                                        #
//##########################################################################

#pragma once
#include <ui_calibdlg.h>
#include <QWidget>
#include <QString>

class QLabel;
//! Dialog for the PCV plugin
class CalibDlg : public QDialog, public Ui::CalibDialog {
 public:
  explicit CalibDlg(QWidget *parent = nullptr);
  QLabel *saveFileNameLabel;
  QString openFilesPath;
 private slots:
  void setSaveFileName();
};
