#include <ros/package.h>

#include <QApplication>
#include <QDebug>
#include <QFile>
#include <QIcon>
#include <QLatin1String>
#include <QMainWindow>
#include <QMessageBox>
#include <QtGui>

#include "remote_ops/remote_console.h"
#include "ros/ros.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "remote_console");
  ros::NodeHandle nh;

  QApplication app(argc, argv);

  MainWindow main_window;

  std::string aios_gui_package_path = ros::package::getPath("aios_manager");

  QCoreApplication::setApplicationName("Remote Operations");
  QIcon appIcon();
  app.setWindowIcon(QIcon("://remoteops_logo"));

  QFontDatabase::addApplicationFont(
      QString::fromStdString(aios_gui_package_path + "/resources/fonts/ProximaNova-Regular.otf"));
  QFontDatabase::addApplicationFont(QString::fromStdString(
      aios_gui_package_path + "/resources/fonts/Proxima Nova Alt Light.otf"));
  QFontDatabase::addApplicationFont(
      QString::fromStdString(aios_gui_package_path + "/resources/fonts/Proxima Nova Alt Thin.otf"));
  QFontDatabase::addApplicationFont(
      QString::fromStdString(aios_gui_package_path + "/resources/fonts/Proxima Nova Black.otf"));
  QFontDatabase::addApplicationFont(
      QString::fromStdString(aios_gui_package_path + "/resources/fonts/Proxima Nova Bold.otf"));
  QFontDatabase::addApplicationFont(QString::fromStdString(
      aios_gui_package_path + "/resources/fonts/Proxima Nova Extrabold.otf"));
  QFontDatabase::addApplicationFont(
      QString::fromStdString(aios_gui_package_path + "/resources/fonts/Proxima Nova Thin.otf"));

  // Set & apply the app style sheet
  //    QFile styleFile( QString::fromStdString(aios_gui_package_path+"/resources/qss/aios.qss") );
  //    std::cout<<"*************"<<aios_gui_package_path+"/resources/qss/aios.qss"<<std::endl;

  //    styleFile.open( QFile::ReadOnly );
  //    QString style(styleFile.readAll());
  //    app.setStyleSheet(style);

  main_window.show();

  return app.exec();
}
