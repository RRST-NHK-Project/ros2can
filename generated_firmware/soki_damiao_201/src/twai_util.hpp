/*====================================================================
<twai_util.hpp>
・独立デバイス型のモータドライバ(MODE_DAMIAO / MODE_ROBSTRIDE)が共通で使う
  CAN(TWAI)の補助関数

  以前は damiao.cpp と robstride.cpp に同じ処理がコピーされていたのを、1か所に
  まとめた(2026-11-06、PRレビュー指摘)。MODE_ROBOMAS / MODE_CUBEMARS / can_task は
  それぞれ独自の処理のまま(実機での再確認が必要になるため、今回は触っていない)。

  ・ログはシリアルに出さない: serialTaskのバイナリフレーム(PCとの通信)と同じUARTに
    混ざると、PC側のフレーム同期を壊すため。失敗は回数だけ数える。
  ・送信は短く待つ(twaiSendのtimeout_ms): モータの電源が切れているなどでACKが返らず
    送信キューが詰まった時に、制御ループが長く止まらないようにするため。
====================================================================*/

#pragma once

#include "driver/twai.h"
#include <Arduino.h>

// TWAIドライバを1Mbps・全ID受信で入れて開始する。rx_queue_len は受信キューの長さ
// (0なら既定値)。失敗したらfalse。
bool twaiInstall(uint32_t rx_queue_len);

// Bus-Off・停止からの復帰と、ESP-IDF TWAIの送信停止(espressif/esp-idf#19068)対策。
// 制御ループから毎回呼ぶ(中で100msごとに間引く)。
void twaiRecoverIfNeeded();

// 1フレーム送る。送信キューに空きが無ければ最大 timeout_ms 待ち、それでも入らなければ
// 捨てて false を返す(失敗回数を数える)。
bool twaiSend(const twai_message_t &msg, uint32_t timeout_ms);

// 起動からの送信失敗回数と、送信停止対策でドライバを入れ直した回数(調査用)。
uint32_t twaiSendFailCount();
uint32_t twaiReinstallCount();
