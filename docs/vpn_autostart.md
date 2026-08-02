# Raspberry Pi VPN autostart

The Raspberry Pi uses the NetworkManager profile `Synology OpenVPN`. The VPN
is kept online by two cooperating components:

- `90-uwb-openvpn-autostart` starts an immediate check after a Wi-Fi, Ethernet,
  or VPN state change;
- `uwb-openvpn-watchdog.timer` runs the same check every 30 seconds, including
  after a transient authentication failure or a stale tunnel.

The watchdog prefers `eth0` when Ethernet has an IPv4 default route and can
reach the OpenVPN server. Otherwise it uses `wlan0`. It discovers the current
`tun` interface from the route and verifies the tunnel by pinging
`192.168.123.1` through it; the presence of a tunnel interface by itself is not
considered healthy.

IPv6 is disabled in the NetworkManager profiles and through sysctl. The
watchdog reapplies the setting to `wlan0`, `eth0`, and every current `tun`
interface whenever it runs.

The OpenVPN profile also declares `AES-256-CBC` in `data-ciphers` for Synology
compatibility and enables `remote-cert-tls=server`. Its username/password are
stored in the root-owned NetworkManager profile (`0600`) so startup does not
depend on a desktop keyring or an interactive login.

Installed files are sourced from `tools/vpn/` and `tools/systemd/` in this
repository. The live locations are `/usr/local/sbin/`,
`/etc/NetworkManager/dispatcher.d/`, `/etc/sysctl.d/`, and
`/etc/systemd/system/`.

The service waits only for NetworkManager, not for `network-online.target`.
When DHCP is not ready yet it exits cleanly; the dispatcher and the 30-second
timer retry without creating a boot dependency cycle.

Useful checks:

```sh
systemctl status uwb-openvpn-watchdog.timer
systemctl status uwb-openvpn-watchdog.service
journalctl -t uwb-openvpn-watchdog -b
nmcli -t -f NAME,TYPE,DEVICE connection show --active
ping -4 -I tun0 -c 2 192.168.123.1
```
