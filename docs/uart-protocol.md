# UART 通信プロトコル

[README に戻る](../README.md)

## 通信設定

| 項目 | 設定 |
| --- | --- |
| ペリフェラル | USART2 |
| ボーレート | 115200 bps |
| データ長 | 8 bit |
| パリティ | なし |
| ストップビット | 1 |
| フロー制御 | なし |
| MCU ピン | TX: PA2、RX: PA15 |

## 受信フレーム（ホスト → RollerMD）

**固定長 10 バイト**です。改行や終端文字は付けません。

| オフセット | サイズ | データ | 備考 |
| --- | ---: | --- | --- |
| 0–4 | 5 B | ASCII `START` | 固定ヘッダ：`53 54 41 52 54`（hex） |
| 5–8 | 4 B | `float32` 角速度指令 [rad/s] | IEEE 754 単精度、little-endian |
| 9 | 1 B | XOR チェックサム | オフセット 0～8 の全 9 バイトを XOR |

受信割り込みでは先頭の `START` を検出した後、残りの 5 バイトを読み取り、XOR が一致した場合に指令用変数 `ref` を更新します。XOR 不一致の場合は指令値を更新せず、次のヘッダを待ちます。

### フレーム例

| 指令 | 送信する 10 バイト（16 進数） |
| --- | --- |
| 0 rad/s | `53 54 41 52 54 00 00 00 00 40` |
| 1 rad/s | `53 54 41 52 54 00 00 80 3F FF` |
| `+INFINITY`（内部リセット扱い） | `53 54 41 52 54 00 00 80 7F BF` |

Python によるフレーム生成例：

~~~python
import struct

def make_frame(speed_rad_s: float) -> bytes:
    payload = b"START" + struct.pack("<f", speed_rad_s)
    checksum = 0
    for byte in payload:
        checksum ^= byte
    return payload + bytes([checksum])

print(make_frame(1.0).hex(" "))
# 53 54 41 52 54 00 00 80 3f ff
~~~

### リセット指令と安全上の注意

受信した `float32` が正の無限大（`+INFINITY`）である場合、制御関数が内部状態の一部をリセットします。**これはハードウェア安全停止ではありません。** 外乱補償項は別に加算されるため、瞬間的にゼロ PWM となる保証はありません。

ゼロ指令でも同様に外乱補償が残る可能性があります。通信が途絶えても、最後に受信した指令値が自動でゼロになるタイムアウト処理はありません。通常運用時は有限の数値を送信し、停止・非常停止は独立した安全機構で担保してください。

## 送信ログ（RollerMD → ホスト）

`Core/Src/main.c` では、約 100 ms ごとに次の形式で状態値をテキスト送信します。

~~~text
ref=<value>, error=<value>, ierror=<value>, output=<value>, d_hat=<value>
~~~

各行は CRLF（`\r\n`）で終わります。

- `ref`：角速度指令 [rad/s]
- `error`：角速度偏差 [rad/s]
- `ierror`：偏差の積分値
- `output`：外乱補償を加えた操作量（PWM 指令相当の数値）
- `d_hat`：推定外乱（操作量換算）

このログは受信指令に対する ACK や正常受信の確認応答ではありません。
