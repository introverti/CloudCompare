/***
 * @Copyright [2022] <Innovusion Inc.>
 * @LastEditTime: 2022-07-27 14:28:18
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

#include "evaluatedlg.h"

EvaluateDlg::EvaluateDlg(QWidget* parent)
    : QDialog(parent, Qt::Tool), Ui::EvaluateDialog() {
  setupUi(this);
}
