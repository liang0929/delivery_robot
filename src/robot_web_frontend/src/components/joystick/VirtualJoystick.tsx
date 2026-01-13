import { useRef, useEffect, useCallback, useState } from 'react';
import { rosbridgeService } from '../../services/rosbridge.service';
import { ROBOT_CONFIG } from '../../config/robot.config';
import styles from './VirtualJoystick.module.css';

interface JoystickState {
  x: number;
  y: number;
}

export function VirtualJoystick() {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [joystick, setJoystick] = useState<JoystickState>({ x: 0, y: 0 });
  const [isActive, setIsActive] = useState(false);
  const publishIntervalRef = useRef<number | null>(null);
  // Use ref to avoid recreating interval on every state change
  const joystickRef = useRef<JoystickState>({ x: 0, y: 0 });

  const canvasSize = 200;
  const baseRadius = 80;
  const stickRadius = 30;

  // Draw joystick
  const draw = useCallback((x: number, y: number) => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    const centerX = canvasSize / 2;
    const centerY = canvasSize / 2;

    // Clear canvas
    ctx.clearRect(0, 0, canvasSize, canvasSize);

    // Draw base circle
    ctx.beginPath();
    ctx.arc(centerX, centerY, baseRadius, 0, Math.PI * 2);
    ctx.fillStyle = '#2a2a2a';
    ctx.fill();
    ctx.strokeStyle = '#444';
    ctx.lineWidth = 2;
    ctx.stroke();

    // Draw cross lines
    ctx.beginPath();
    ctx.moveTo(centerX - baseRadius, centerY);
    ctx.lineTo(centerX + baseRadius, centerY);
    ctx.moveTo(centerX, centerY - baseRadius);
    ctx.lineTo(centerX, centerY + baseRadius);
    ctx.strokeStyle = '#333';
    ctx.lineWidth = 1;
    ctx.stroke();

    // Calculate stick position (clamped to base)
    const stickX = centerX + x * baseRadius;
    const stickY = centerY - y * baseRadius; // Invert Y for screen coords

    // Draw stick
    ctx.beginPath();
    ctx.arc(stickX, stickY, stickRadius, 0, Math.PI * 2);
    const gradient = ctx.createRadialGradient(stickX, stickY, 0, stickX, stickY, stickRadius);
    gradient.addColorStop(0, '#6a6a6a');
    gradient.addColorStop(1, '#4a4a4a');
    ctx.fillStyle = gradient;
    ctx.fill();
    ctx.strokeStyle = isActive ? '#00ff88' : '#555';
    ctx.lineWidth = 2;
    ctx.stroke();
  }, [isActive]);

  // Handle input
  const handleInput = useCallback((clientX: number, clientY: number) => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const rect = canvas.getBoundingClientRect();
    const centerX = rect.left + canvasSize / 2;
    const centerY = rect.top + canvasSize / 2;

    let dx = (clientX - centerX) / baseRadius;
    let dy = -(clientY - centerY) / baseRadius; // Invert Y

    // Clamp to unit circle
    const magnitude = Math.sqrt(dx * dx + dy * dy);
    if (magnitude > 1) {
      dx /= magnitude;
      dy /= magnitude;
    }

    joystickRef.current = { x: dx, y: dy };
    setJoystick({ x: dx, y: dy });
  }, []);

  // Mouse/Touch handlers
  const handleStart = useCallback((e: React.MouseEvent | React.TouchEvent) => {
    e.preventDefault();
    setIsActive(true);

    const { clientX, clientY } = 'touches' in e ? e.touches[0] : e;
    handleInput(clientX, clientY);
  }, [handleInput]);

  const handleMove = useCallback((e: React.MouseEvent | React.TouchEvent) => {
    if (!isActive) return;
    e.preventDefault();

    const { clientX, clientY } = 'touches' in e ? e.touches[0] : e;
    handleInput(clientX, clientY);
  }, [isActive, handleInput]);

  const handleEnd = useCallback(() => {
    setIsActive(false);
    joystickRef.current = { x: 0, y: 0 };
    setJoystick({ x: 0, y: 0 });
    // Immediately send stop command
    rosbridgeService.publishCmdVel(0, 0);
  }, []);

  // Draw effect
  useEffect(() => {
    draw(joystick.x, joystick.y);
  }, [joystick, draw]);

  // Publish cmd_vel at fixed rate - use ref to avoid recreating interval
  useEffect(() => {
    publishIntervalRef.current = window.setInterval(() => {
      const { x, y } = joystickRef.current;
      const linear = y * ROBOT_CONFIG.MAX_LINEAR_VEL;
      const angular = -x * ROBOT_CONFIG.MAX_ANGULAR_VEL;
      rosbridgeService.publishCmdVel(linear, angular);
    }, 1000 / ROBOT_CONFIG.CMD_VEL_RATE);

    return () => {
      if (publishIntervalRef.current) {
        clearInterval(publishIntervalRef.current);
      }
    };
  }, []); // Empty deps - interval created once on mount

  // Global mouse/touch up handlers
  useEffect(() => {
    const handleGlobalEnd = () => {
      if (isActive) {
        handleEnd();
      }
    };

    window.addEventListener('mouseup', handleGlobalEnd);
    window.addEventListener('touchend', handleGlobalEnd);

    return () => {
      window.removeEventListener('mouseup', handleGlobalEnd);
      window.removeEventListener('touchend', handleGlobalEnd);
    };
  }, [isActive, handleEnd]);

  return (
    <div className={styles.container}>
      <canvas
        ref={canvasRef}
        width={canvasSize}
        height={canvasSize}
        className={styles.canvas}
        onMouseDown={handleStart}
        onMouseMove={handleMove}
        onTouchStart={handleStart}
        onTouchMove={handleMove}
      />
      <div className={styles.info}>
        <div>Linear: {(joystick.y * ROBOT_CONFIG.MAX_LINEAR_VEL).toFixed(3)} m/s</div>
        <div>Angular: {(-joystick.x * ROBOT_CONFIG.MAX_ANGULAR_VEL).toFixed(3)} rad/s</div>
      </div>
    </div>
  );
}
