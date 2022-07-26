/***
 * @Copyright [2022] <Innovusion Inc.>
 * @LastEditTime: 2022-07-22 15:31:36
 * @LastEditors: Tianyun Xuan
 */
/***
 * @Copyright [2022] <Innovusion Inc.>
 * @LastEditTime: 2022-07-21 14:52:20
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

#include "calibdlg.h"

#include <QtWidgets>

CalibDlg::CalibDlg(QWidget *parent)
    : QDialog(parent, Qt::Tool), Ui::CalibDialog() {
  setupUi(this);
  int frameStyle = QFrame::Sunken | QFrame::Panel;
  saveFileNameLabel = new QLabel;
  saveFileNameLabel->setFrameStyle(frameStyle);
  QPushButton *saveFileNameButton = new QPushButton(tr("SaveFileName"));

  connect(saveFileNameButton, &QAbstractButton::clicked, this,
          &CalibDlg::setSaveFileName);
  QGridLayout *layout = new QGridLayout(this);
  layout->addWidget(saveFileNameButton, 0, 0);
  layout->addWidget(saveFileNameLabel, 0, 1);
  layout->addWidget(lidar_type_label, 1, 0, Qt::AlignCenter);
  layout->addWidget(lidar_type_box, 1, 1);
  layout->addWidget(lidar_field_label, 2, 0, Qt::AlignCenter);
  layout->addWidget(lidar_field_box, 2, 1);
  layout->addWidget(buttonBox);
}

void CalibDlg::setSaveFileName() {
  const QFileDialog::Options options = QFlag(0);
  QString selectedFilter;
  QString fileName = QFileDialog::getSaveFileName(
      this, tr("QFileDialog::getSaveFileName()"), saveFileNameLabel->text(),
      tr("All Files (*);;Text Files (*.txt)"), &selectedFilter, options);
  if (!fileName.isEmpty()) {
    saveFileNameLabel->setText(fileName);
    openFilesPath = fileName;
  }
}
