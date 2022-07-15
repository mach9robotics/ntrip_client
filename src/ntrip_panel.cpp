/**
 * Mach9 Robotics, Inc
 *
 * RViz panel for ntrip client
 *
 * Written by Haowen Shi <shi@mach9.io>, Jul 2022
 */

#include <pluginlib/class_list_macros.h>

#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>

#include "ntrip_client/ntrip_panel.hpp"

static const QString kHostText = QStringLiteral("Host: ");
static const QString kPortText = QStringLiteral("Port: ");
static const QString kMountpointText = QStringLiteral("Mountpoint: ");
static const QString kUsernameText = QStringLiteral("Username: ");
static const QString kPasswordText = QStringLiteral("Password: ");
static const QString kNtripVersionText = QStringLiteral("Ntrip Version: ");
static const QString kConnectText = QStringLiteral("Connect");
static const QString kDisconnectText = QStringLiteral("Disconnect");

namespace ntrip_client
{

NtripPanel::NtripPanel(QWidget* parent)
    : rviz::Panel(parent)
{
    // setup ui
    m_host_textfield = new QLineEdit(this);
    m_port_textfield = new QLineEdit(this);
    m_mountpoint_textfield = new QLineEdit(this);
    m_username_textfield = new QLineEdit(this);
    m_password_textfield = new QLineEdit(this);
    m_ntrip_version_textfield = new QLineEdit(this);
    m_connect_button = new QPushButton(kConnectText, this);
    m_disconnect_button = new QPushButton(kDisconnectText, this);

    QHBoxLayout* host_layout = new QHBoxLayout;
    host_layout->addWidget(new QLabel(kHostText, this));
    host_layout->addWidget(m_host_textfield);

    QHBoxLayout* port_layout = new QHBoxLayout;
    port_layout->addWidget(new QLabel(kPortText, this));
    port_layout->addWidget(m_port_textfield);

    QHBoxLayout* mountpoint_layout = new QHBoxLayout;
    mountpoint_layout->addWidget(new QLabel(kMountpointText, this));
    mountpoint_layout->addWidget(m_mountpoint_textfield);

    QHBoxLayout* username_layout = new QHBoxLayout;
    username_layout->addWidget(new QLabel(kUsernameText, this));
    username_layout->addWidget(m_username_textfield);

    QHBoxLayout* password_layout = new QHBoxLayout;
    password_layout->addWidget(new QLabel(kPasswordText, this));
    password_layout->addWidget(m_password_textfield);

    QHBoxLayout* ntrip_version_layout = new QHBoxLayout;
    ntrip_version_layout->addWidget(new QLabel(kNtripVersionText, this));
    ntrip_version_layout->addWidget(m_ntrip_version_textfield);

    QHBoxLayout* buttons_layout = new QHBoxLayout;
    buttons_layout->addWidget(m_connect_button);
    buttons_layout->addWidget(m_disconnect_button);

    QVBoxLayout* layout = new QVBoxLayout;
    layout->addLayout(host_layout);
    layout->addLayout(port_layout);
    layout->addLayout(mountpoint_layout);
    layout->addLayout(username_layout);
    layout->addLayout(password_layout);
    layout->addLayout(ntrip_version_layout);
    layout->addLayout(buttons_layout);

    setLayout(layout);

    // setup service client
    m_connect_service_client = m_nh.serviceClient<ntrip_client::NtripClientConnect>("/ntrip_client_connect");

    connect(m_connect_button, SIGNAL(clicked()), this, SLOT(connect_clicked()));
    connect(m_disconnect_button, SIGNAL(clicked()), this, SLOT(disconnect_clicked()));
}

NtripPanel::~NtripPanel()
{
    return;
}

void NtripPanel::save(rviz::Config config) const
{
    rviz::Panel::save(config);
    config.mapSetValue("host", m_host_textfield->text().toStdString().c_str());
    config.mapSetValue("port", m_port_textfield->text().toStdString().c_str());
    config.mapSetValue("mountpoint", m_mountpoint_textfield->text().toStdString().c_str());
    config.mapSetValue("username", m_username_textfield->text().toStdString().c_str());
    config.mapSetValue("password", m_password_textfield->text().toStdString().c_str());
    config.mapSetValue("ntrip_version", m_ntrip_version_textfield->text().toStdString().c_str());
}

void NtripPanel::load(const rviz::Config& config)
{
    rviz::Panel::load(config);
    QString save_val;
    if (config.mapGetString("host", &save_val)) m_host_textfield->setText(save_val);
    if (config.mapGetString("port", &save_val)) m_port_textfield->setText(save_val);
    if (config.mapGetString("mountpoint", &save_val)) m_mountpoint_textfield->setText(save_val);
    if (config.mapGetString("username", &save_val)) m_username_textfield->setText(save_val);
    if (config.mapGetString("password", &save_val)) m_password_textfield->setText(save_val);
    if (config.mapGetString("ntrip_version", &save_val)) m_ntrip_version_textfield->setText(save_val);
}

void NtripPanel::connect_clicked()
{
    ntrip_client::NtripClientConnect srv;
    srv.request.is_connect = true;
    srv.request.authenticate = true;
    srv.request.host = m_host_textfield->text().toStdString();
    srv.request.port = m_port_textfield->text().toStdString();
    srv.request.mountpoint = m_mountpoint_textfield->text().toStdString();
    srv.request.username = m_username_textfield->text().toStdString();
    srv.request.password = m_password_textfield->text().toStdString();
    srv.request.ntrip_version = m_ntrip_version_textfield->text().toStdString();
    if (m_connect_service_client.call(srv))
    {
        ROS_INFO("NtripClientConnect: %s", srv.response.success ? "success" : "failure");
    }
    else
    {
        ROS_ERROR("Failed to call service NtripClientConnect");
    }
    return;
}

void NtripPanel::disconnect_clicked()
{
    ntrip_client::NtripClientConnect srv;
    srv.request.is_connect = false;
    if (m_connect_service_client.call(srv))
    {
        ROS_INFO("NtripClientConnect: %s", srv.response.success ? "success" : "failure");
    }
    else
    {
        ROS_ERROR("Failed to call service NtripClientConnect");
    }
}

} // namespace ntrip_client

PLUGINLIB_EXPORT_CLASS(ntrip_client::NtripPanel, rviz::Panel)
