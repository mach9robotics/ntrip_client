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
#include <QTimer>
#include <QCheckBox>
#include <QFormLayout>

#include "ntrip_client/ntrip_panel.hpp"
#include "ntrip_client/NtripClientConnect.h"
#include "ntrip_client/NtripClientSettings.h"
#include "ntrip_client/NtripClientStatus.h"

static const QString kHostText = QStringLiteral("Host: ");
static const QString kPortText = QStringLiteral("Port: ");
static const QString kMountpointText = QStringLiteral("Mountpoint: ");
static const QString kUsernameText = QStringLiteral("Username: ");
static const QString kPasswordText = QStringLiteral("Password: ");
static const QString kNtripVersionText = QStringLiteral("Ntrip Version: ");
static const QString kNmeaUpText = QStringLiteral("Base Roaming: ");
static const QString kConnectText = QStringLiteral("Connect");
static const QString kDisconnectText = QStringLiteral("Disconnect");

static const QString kStatusInvalidText = QStringLiteral("Unknown");
static const QString kStatusConnectedText = QStringLiteral("Connected");
static const QString kStatusDisconnectedText = QStringLiteral("Disconnected");

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
    m_nmea_up_checkbox = new QCheckBox(this);
    m_connect_button = new QPushButton(kConnectText, this);
    m_disconnect_button = new QPushButton(kDisconnectText, this);
    m_status_label = new QLabel(this);
    m_status_timer = new QTimer(this);

    set_state(NtripClientState::INVALID);

    QFormLayout* config_layout = new QFormLayout;
    config_layout->addRow(kHostText, m_host_textfield);
    config_layout->addRow(kPortText, m_port_textfield);
    config_layout->addRow(kMountpointText, m_mountpoint_textfield);
    config_layout->addRow(kUsernameText, m_username_textfield);
    config_layout->addRow(kPasswordText, m_password_textfield);

    QHBoxLayout* ntrip_version_layout = new QHBoxLayout;
    ntrip_version_layout->addWidget(new QLabel(kNtripVersionText, this));
    ntrip_version_layout->addWidget(m_ntrip_version_textfield);
    ntrip_version_layout->addWidget(new QLabel(kNmeaUpText, this));
    ntrip_version_layout->addWidget(m_nmea_up_checkbox);

    QHBoxLayout* buttons_layout = new QHBoxLayout;
    buttons_layout->addWidget(m_connect_button);
    buttons_layout->addWidget(m_disconnect_button);

    QVBoxLayout* layout = new QVBoxLayout;
    layout->addLayout(config_layout);
    layout->addLayout(ntrip_version_layout);
    layout->addLayout(buttons_layout);

    layout->addWidget(m_status_label);

    setLayout(layout);

    // setup service client
    m_connect_service_client = m_nh.serviceClient<ntrip_client::NtripClientConnect>("/ntrip_client_connect");
    m_settings_client = m_nh.serviceClient<ntrip_client::NtripClientSettings>("/ntrip_client_settings");
    m_ntrip_status_client = m_nh.serviceClient<ntrip_client::NtripClientStatus>("/ntrip_client_status");

    connect(m_connect_button, SIGNAL(clicked()), this, SLOT(connect_clicked()));
    connect(m_disconnect_button, SIGNAL(clicked()), this, SLOT(disconnect_clicked()));
    connect(m_nmea_up_checkbox, SIGNAL(clicked()), this, SLOT(nmea_up_clicked()));
    connect(m_status_timer, SIGNAL(timeout()), this, SLOT(update_ntrip_status()));

    m_status_timer->start(1000);
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
    config.mapSetValue("nmea_up", m_nmea_up_checkbox->isChecked());
}

void NtripPanel::load(const rviz::Config& config)
{
    rviz::Panel::load(config);
    QString save_val;
    bool save_bool;
    if (config.mapGetString("host", &save_val)) m_host_textfield->setText(save_val);
    if (config.mapGetString("port", &save_val)) m_port_textfield->setText(save_val);
    if (config.mapGetString("mountpoint", &save_val)) m_mountpoint_textfield->setText(save_val);
    if (config.mapGetString("username", &save_val)) m_username_textfield->setText(save_val);
    if (config.mapGetString("password", &save_val)) m_password_textfield->setText(save_val);
    if (config.mapGetString("ntrip_version", &save_val)) m_ntrip_version_textfield->setText(save_val);
    if (config.mapGetBool("nmea_up", &save_bool)) m_nmea_up_checkbox->setChecked(save_bool);

    // auto trigger once based on saved connection history
    connect_clicked();
    nmea_up_clicked();
}

void NtripPanel::connect_clicked()
{
    ntrip_client::NtripClientConnect srv;
    srv.request.is_connect = true;
    srv.request.keep_alive = true;
    srv.request.authenticate = true;
    srv.request.host = m_host_textfield->text().toStdString();
    srv.request.port = m_port_textfield->text().toStdString();
    srv.request.mountpoint = m_mountpoint_textfield->text().toStdString();
    srv.request.username = m_username_textfield->text().toStdString();
    srv.request.password = m_password_textfield->text().toStdString();
    srv.request.ntrip_version = m_ntrip_version_textfield->text().toStdString();
    if (!m_connect_service_client.call(srv))
    {
        ROS_ERROR("Failed to call service NtripClientConnect");
    }
    return;
}

void NtripPanel::disconnect_clicked()
{
    ntrip_client::NtripClientConnect srv;
    srv.request.is_connect = false;
    if (!m_connect_service_client.call(srv))
    {
        ROS_ERROR("Failed to call service NtripClientConnect");
    }
}

void NtripPanel::update_ntrip_status()
{
    ntrip_client::NtripClientStatus srv;
    if (m_ntrip_status_client.call(srv))
    {
        set_state(srv.response.status == 0 ? NtripClientState::CONNECTED : NtripClientState::DISCONNECTED);
    }
    else
    {
        set_state(NtripClientState::INVALID);
        ROS_ERROR_ONCE("Failed to call service NtripClientStatus");
    }
}

void NtripPanel::nmea_up_clicked()
{
    ntrip_client::NtripClientSettings srv;
    srv.request.nmea_up = m_nmea_up_checkbox->isChecked();
    srv.request.nmea_topic = "/zed_f9p/nmea";
    if (!m_settings_client.call(srv))
    {
        ROS_ERROR("Failed to call service NtripClientSettings");
    }
}

void NtripPanel::set_state(NtripClientState state)
{
    switch (state)
    {
        case NtripClientState::INVALID:
            m_status_label->setText(kStatusInvalidText);
            m_status_label->setStyleSheet("QLabel { background-color : yellow; color : black; }");
            break;
        case NtripClientState::CONNECTED:
            m_status_label->setText(kStatusConnectedText);
            m_status_label->setStyleSheet("QLabel { background-color : green; color : white; }");
            break;
        case NtripClientState::DISCONNECTED:
            m_status_label->setText(kStatusDisconnectedText);
            m_status_label->setStyleSheet("QLabel { background-color : red; color : white; }");
            break;
    }
}

} // namespace ntrip_client

PLUGINLIB_EXPORT_CLASS(ntrip_client::NtripPanel, rviz::Panel)
