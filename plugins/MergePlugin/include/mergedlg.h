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
#include <ui_mergedlg.h>

//! Dialog for the PCV plugin
class MergeDlg : public QDialog, public Ui::MergeDialog {
 public:
  explicit MergeDlg(QWidget* parent = nullptr);
};
