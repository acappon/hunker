#include "ControllerLinkProbe.h"

#include <dbus/dbus.h>

#include <cstring>
#include <map>
#include <strings.h>
#include <vector>

namespace
{
const char *kBatteryLevelUuid = "00002a19-0000-1000-8000-00805f9b34fb";
const char *kHidServiceUuid = "00001812-0000-1000-8000-00805f9b34fb";

int64_t steadyNowNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// Read a variant holding a string or a bool. Returns false if it holds something else.
bool variantString(DBusMessageIter *variant, std::string &out)
{
    DBusMessageIter v;
    dbus_message_iter_recurse(variant, &v);
    if (dbus_message_iter_get_arg_type(&v) != DBUS_TYPE_STRING)
        return false;
    const char *s = nullptr;
    dbus_message_iter_get_basic(&v, &s);
    out = s ? s : "";
    return true;
}

bool variantBool(DBusMessageIter *variant, bool &out)
{
    DBusMessageIter v;
    dbus_message_iter_recurse(variant, &v);
    if (dbus_message_iter_get_arg_type(&v) != DBUS_TYPE_BOOLEAN)
        return false;
    dbus_bool_t b = FALSE;
    dbus_message_iter_get_basic(&v, &b);
    out = b;
    return true;
}

bool variantStringArrayContains(DBusMessageIter *variant, const char *wanted)
{
    DBusMessageIter v, arr;
    dbus_message_iter_recurse(variant, &v);
    if (dbus_message_iter_get_arg_type(&v) != DBUS_TYPE_ARRAY)
        return false;
    dbus_message_iter_recurse(&v, &arr);
    while (dbus_message_iter_get_arg_type(&arr) == DBUS_TYPE_STRING)
    {
        const char *s = nullptr;
        dbus_message_iter_get_basic(&arr, &s);
        if (s && strcasecmp(s, wanted) == 0)
            return true;
        dbus_message_iter_next(&arr);
    }
    return false;
}
} // namespace

ControllerLinkProbe::~ControllerLinkProbe()
{
    stop();
}

void ControllerLinkProbe::start()
{
    if (m_thread.joinable())
        return;
    m_stop = false;
    m_thread = std::thread(&ControllerLinkProbe::threadMain, this);
}

void ControllerLinkProbe::stop()
{
    m_stop = true;
    if (m_thread.joinable())
        m_thread.join();   // at most one read in flight, so this waits <= kCallTimeoutMs
}

double ControllerLinkProbe::secondsSinceLastReply() const
{
    const int64_t last = m_lastReplyNs.load();
    if (last == 0)
        return 1e9;
    return (steadyNowNs() - last) * 1e-9;
}

std::string ControllerLinkProbe::status() const
{
    std::lock_guard<std::mutex> lock(m_statusMutex);
    return m_status;
}

void ControllerLinkProbe::setStatus(const std::string &s)
{
    std::lock_guard<std::mutex> lock(m_statusMutex);
    m_status = s;
}

void ControllerLinkProbe::threadMain()
{
    DBusError err;
    dbus_error_init(&err);
    m_conn = dbus_bus_get_private(DBUS_BUS_SYSTEM, &err);
    if (!m_conn)
    {
        setStatus(std::string("no system D-Bus: ") + (err.message ? err.message : "?"));
        dbus_error_free(&err);
        return;   // stays stale forever: robot can't be enabled (fail-safe)
    }
    dbus_connection_set_exit_on_disconnect(m_conn, FALSE);

    auto nextLookup = std::chrono::steady_clock::now();
    while (!m_stop)
    {
        const auto tickStart = std::chrono::steady_clock::now();

        if (m_charPath.empty())
        {
            // Look for the controller at most once a second.
            if (tickStart >= nextLookup)
            {
                nextLookup = tickStart + std::chrono::seconds(1);
                if (findCharacteristic())
                    setStatus("probing " + m_charPath);
                else
                    setStatus("no connected HID device with a Battery Level characteristic");
            }
        }
        else
        {
            std::string error;
            if (readOnce(error))
            {
                m_lastReplyNs = steadyNowNs();
                setStatus("OK, replies from " + m_charPath);
            }
            else
            {
                setStatus("read failed: " + error);
                // Object gone (controller disconnected or re-paired): look it up again.
                if (error.find("UnknownObject") != std::string::npos ||
                    error.find("UnknownMethod") != std::string::npos ||
                    error.find("NotConnected") != std::string::npos ||
                    error.find("Not connected") != std::string::npos)
                {
                    m_charPath.clear();
                }
            }
        }

        std::this_thread::sleep_until(tickStart + std::chrono::milliseconds(kPeriodMs));
    }

    dbus_connection_close(m_conn);
    dbus_connection_unref(m_conn);
    m_conn = nullptr;
}

bool ControllerLinkProbe::readOnce(std::string &error)
{
    DBusMessage *msg = dbus_message_new_method_call(
        "org.bluez", m_charPath.c_str(), "org.bluez.GattCharacteristic1", "ReadValue");
    if (!msg)
    {
        error = "out of memory";
        return false;
    }
    // ReadValue(a{sv} options) with no options.
    DBusMessageIter args, dict;
    dbus_message_iter_init_append(msg, &args);
    dbus_message_iter_open_container(&args, DBUS_TYPE_ARRAY, "{sv}", &dict);
    dbus_message_iter_close_container(&args, &dict);

    DBusError err;
    dbus_error_init(&err);
    DBusMessage *reply = dbus_connection_send_with_reply_and_block(m_conn, msg, kCallTimeoutMs, &err);
    dbus_message_unref(msg);
    if (!reply)
    {
        error = std::string(err.name ? err.name : "?") + " " + (err.message ? err.message : "");
        dbus_error_free(&err);
        return false;
    }
    dbus_message_unref(reply);
    return true;
}

bool ControllerLinkProbe::findCharacteristic()
{
    DBusMessage *msg = dbus_message_new_method_call(
        "org.bluez", "/", "org.freedesktop.DBus.ObjectManager", "GetManagedObjects");
    if (!msg)
        return false;
    DBusError err;
    dbus_error_init(&err);
    DBusMessage *reply = dbus_connection_send_with_reply_and_block(m_conn, msg, 2000, &err);
    dbus_message_unref(msg);
    if (!reply)
    {
        dbus_error_free(&err);
        return false;
    }

    struct Device
    {
        bool connected = false;
        bool hid = false;
    };
    std::map<std::string, Device> devices;                  // device path -> info
    std::vector<std::string> batteryChars;                  // characteristic paths

    // a{oa{sa{sv}}}
    DBusMessageIter top, objects;
    dbus_message_iter_init(reply, &top);
    if (dbus_message_iter_get_arg_type(&top) == DBUS_TYPE_ARRAY)
    {
        dbus_message_iter_recurse(&top, &objects);
        while (dbus_message_iter_get_arg_type(&objects) == DBUS_TYPE_DICT_ENTRY)
        {
            DBusMessageIter objEntry, ifaces;
            dbus_message_iter_recurse(&objects, &objEntry);
            const char *path = nullptr;
            dbus_message_iter_get_basic(&objEntry, &path);
            dbus_message_iter_next(&objEntry);
            dbus_message_iter_recurse(&objEntry, &ifaces);

            while (dbus_message_iter_get_arg_type(&ifaces) == DBUS_TYPE_DICT_ENTRY)
            {
                DBusMessageIter ifEntry, props;
                dbus_message_iter_recurse(&ifaces, &ifEntry);
                const char *iface = nullptr;
                dbus_message_iter_get_basic(&ifEntry, &iface);
                dbus_message_iter_next(&ifEntry);
                dbus_message_iter_recurse(&ifEntry, &props);

                const bool isDevice = strcmp(iface, "org.bluez.Device1") == 0;
                const bool isChar = strcmp(iface, "org.bluez.GattCharacteristic1") == 0;
                if (isDevice || isChar)
                {
                    while (dbus_message_iter_get_arg_type(&props) == DBUS_TYPE_DICT_ENTRY)
                    {
                        DBusMessageIter pEntry;
                        dbus_message_iter_recurse(&props, &pEntry);
                        const char *name = nullptr;
                        dbus_message_iter_get_basic(&pEntry, &name);
                        dbus_message_iter_next(&pEntry);   // now at the variant

                        if (isDevice && strcmp(name, "Connected") == 0)
                        {
                            variantBool(&pEntry, devices[path].connected);
                        }
                        else if (isDevice && strcmp(name, "UUIDs") == 0)
                        {
                            devices[path].hid = variantStringArrayContains(&pEntry, kHidServiceUuid);
                        }
                        else if (isChar && strcmp(name, "UUID") == 0)
                        {
                            std::string uuid;
                            if (variantString(&pEntry, uuid) && strcasecmp(uuid.c_str(), kBatteryLevelUuid) == 0)
                                batteryChars.push_back(path);
                        }
                        dbus_message_iter_next(&props);
                    }
                }
                dbus_message_iter_next(&ifaces);
            }
            dbus_message_iter_next(&objects);
        }
    }
    dbus_message_unref(reply);

    // Pick a Battery Level characteristic that belongs to a connected HID device.
    for (const auto &c : batteryChars)
    {
        for (const auto &[devPath, dev] : devices)
        {
            if (dev.connected && dev.hid && c.rfind(devPath + "/", 0) == 0)
            {
                m_charPath = c;
                return true;
            }
        }
    }
    return false;
}
