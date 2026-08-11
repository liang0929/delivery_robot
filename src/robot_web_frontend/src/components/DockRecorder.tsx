// 充電座位置記錄面板：部署時把車推到與充電座對接的位置，按一顆按鈕就把
// 當下位姿寫進 opennav_docking 的 dock database。
//
// 為什麼是兩段式確認而不是 window.confirm：這顆按鈕會覆寫正式的充電座座標，
// 誤按的代價是機器人之後停到錯的位置去充電；但 window.confirm 會阻塞整個
// 分頁（含 WebSocket 狀態更新），手機上的樣式也不受控。改成面板內就地確認，
// 順便把「odom 只是測試」的警告寫在確認當下——那才是使用者真的會讀的時機。

import { useState } from 'react';
import { recordDockPose } from '../api/robot.api';
import { useAction } from '../hooks/useAction';
import type { DockPoseRecord } from '../api/types';
import styles from './DockRecorder.module.css';

type Pending = 'map' | 'odom' | null;

/** 位姿 → 「x=1.23 m, y=-0.45 m, θ=180.0°」 */
function formatPose(p: DockPoseRecord['pose']): string {
  return `x=${p.x_m.toFixed(3)} m, y=${p.y_m.toFixed(3)} m, θ=${p.yaw_deg.toFixed(1)}°`;
}

export function DockRecorder() {
  const { busy, run } = useAction();
  const [pending, setPending] = useState<Pending>(null);
  const [result, setResult] = useState<DockPoseRecord | null>(null);

  const confirm = (frame: 'map' | 'odom') => {
    setPending(null);
    void run(
      frame === 'map' ? '記錄充電座位置' : '記錄充電座位置（odom 測試）',
      async () => {
        const res = await recordDockPose(frame);
        setResult(res);
      },
      frame === 'map' ? '充電座位置已記錄' : '已記錄 odom 測試值（非正式座標）',
    );
  };

  return (
    <section className="panel">
      <h2 className="panelTitle">4. 充電座</h2>
      <p className="hintText">
        把機器人推到與充電座<b>完全對接</b>（刷塊已接觸）的位置，再按下方按鈕，
        系統會把當下位姿換算成充電座座標寫入設定檔。之後的自動回充由這個座標推算。
      </p>

      {pending === null ? (
        <div className="row" style={{ marginTop: 10 }}>
          <button
            type="button"
            className="btn primary"
            disabled={busy !== null}
            onClick={() => setPending('map')}
          >
            記錄充電座位置
          </button>
          <button
            type="button"
            className="btn small"
            disabled={busy !== null}
            onClick={() => setPending('odom')}
          >
            以 odom 記錄（僅測試）
          </button>
        </div>
      ) : (
        <div className={styles.confirmBox}>
          <strong>請確認機器人已與充電座完全對接</strong>
          <p>
            記錄的是<b>此刻</b>的位置與朝向，會覆寫設定檔中的充電座座標。
            車若沒有真的接觸到刷塊，之後的自動回充會停在錯的位置。
          </p>
          {pending === 'odom' && (
            <p className={styles.testWarn}>
              🔴 這是 odom 測試模式：odom 每次重開機都會歸零，記到的值下次開機
              就對不上實體充電座，<b>不可當正式座標使用</b>。正式記錄必須在導航
              模式定位完成後執行。
            </p>
          )}
          <div className="row" style={{ marginTop: 10 }}>
            <button
              type="button"
              className={`btn ${pending === 'map' ? 'primary' : ''}`}
              onClick={() => confirm(pending)}
            >
              {pending === 'map' ? '確認已對接，記錄' : '確認記錄測試值'}
            </button>
            <button type="button" className="btn" onClick={() => setPending(null)}>
              取消
            </button>
          </div>
        </div>
      )}

      {result && (
        <div className={result.test_only ? styles.resultTest : styles.resultOk}>
          {result.test_only ? (
            <strong>⚠️ 僅測試（{result.frame} frame，重開機後失效）</strong>
          ) : (
            <strong>已記錄（{result.frame} frame）</strong>
          )}
          <p className={styles.poseLine}>{formatPose(result.pose)}</p>
          <p className={styles.metaLine}>
            對接時 base_link：{formatPose(result.base_link)}
          </p>
          <p className={styles.metaLine}>記錄時間：{result.recorded_at}</p>
        </div>
      )}
    </section>
  );
}
