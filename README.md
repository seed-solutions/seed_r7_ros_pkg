# 音声つきモーションを使いたい場合

デフォルトではpulse audioサーバが、root実行を受け付けないので、受け付けるようにする。

/etc/systemd/system/pulseaudio.service
を作成して、以下を記載

```
[Unit]
Description=PulseAudio system server

[Service]
Type=notify
ExecStart=pulseaudio --daemonize=no --system --realtime --log-target=journal

[Install]
WantedBy=multi-user.target
```

サービスを無効にする
```
systemctl disable --user pulseaudio.service  pulseaudio.socket
```

サービスを有効にする
```shell
sudo systemctl --system enable pulseaudio.service
```

/etc/pulse/client.confを、以下のように修正
```
default-server = /var/run/pulse/native
autospawn = no
```

pulse audioのユーザグループにアクセスしたいユーザを追加
```shell
sudo adduser root pulse-access
sudo adduser seed pulse-access
```

リブート
```shell
reboot
```

see also
https://stackoverflow.com/questions/66775654/how-can-i-make-pulseaudio-run-as-root

