/**
 * Mach9 Robotics, Inc
 *
 * RViz panel for ntrip client
 *
 * Written by Haowen Shi <shi@mach9.io>, Jul 2022
 */

#ifndef __NTRIP_PANEL_HPP__
#define __NTRIP_PANEL_HPP__

#include <ros/ros.h>
#include <rviz/panel.h>

#include "ntrip_client/NtripClientConnect.h"

class QLineEdit;
class QLabel;
class QPushButton;

namespace ntrip_client
{

class NtripPanel : public rviz::Panel
{
    Q_OBJECT

public:
    NtripPanel(QWidget* parent = 0);
    virtual ~NtripPanel();

    virtual void save(rviz::Config config) const;
    virtual void load(const rviz::Config& config);

Q_SIGNALS:
    // empty

protected Q_SLOTS:
    // ui
    void connect_clicked();

protected:
    QLineEdit* m_host_textfield;
    QLineEdit* m_port_textfield;
    QLineEdit* m_mountpoint_textfield;
    QLineEdit* m_username_textfield;
    QLineEdit* m_password_textfield;
    QLineEdit* m_ntrip_version_textfield;
    QPushButton* m_connect_button;

    ros::NodeHandle m_nh;
    ros::ServiceClient m_connect_service_client;
};

}

#endif /*__NTRIP_PANEL_HPP__ */