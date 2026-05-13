#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
alert_dialog.py - Universal Alert Dialog System

Features:
1. Unified dialog interface
2. Multiple alert levels (INFO, WARNING, ERROR, CRITICAL)
3. Sound alerts
4. Auto-close support
5. Thread-safe, non-blocking for ROS nodes

Usage:
    from alert_dialog import AlertDialog

    # Show warning dialog
    AlertDialog.warning(
        title="Gazebo Convergence Failed",
        message="Gazebo virtual robot failed to converge within timeout",
        details="Max joint error: 0.05 rad (2.86°)\nTimeout: 8.0 seconds"
    )

    # Show error dialog (with sound)
    AlertDialog.error(
        title="Safety Check Failed",
        message="Large motion detected, collision risk",
        details="Joint error: 0.8 rad (45.8°)\nThreshold: 0.5 rad (28.6°)",
        sound=True
    )
"""

import tkinter as tk
from tkinter import ttk
import threading
import subprocess
import os


class AlertDialog:
    """通用弹窗报警系统"""

    # 报警级别
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
    CRITICAL = "CRITICAL"

    # 颜色配置
    COLORS = {
        INFO: {"bg": "#E3F2FD", "fg": "#1976D2", "icon": "ℹ️"},
        WARNING: {"bg": "#FFF3E0", "fg": "#F57C00", "icon": "⚠️"},
        ERROR: {"bg": "#FFEBEE", "fg": "#D32F2F", "icon": "❌"},
        CRITICAL: {"bg": "#F3E5F5", "fg": "#7B1FA2", "icon": "🚨"}
    }

    @staticmethod
    def show(title, message, details=None, level=WARNING, sound=False, auto_close=None):
        """
        显示弹窗

        参数:
            title: 标题
            message: 主要消息
            details: 详细信息（可选）
            level: 报警级别（INFO, WARNING, ERROR, CRITICAL）
            sound: 是否播放声音
            auto_close: 自动关闭时间（秒），None 表示不自动关闭
        """
        # 在新线程中显示弹窗，避免阻塞 ROS 节点
        thread = threading.Thread(
            target=AlertDialog._show_dialog,
            args=(title, message, details, level, sound, auto_close),
            daemon=True
        )
        thread.start()

    @staticmethod
    def _show_dialog(title, message, details, level, sound, auto_close):
        """内部方法：显示弹窗"""
        # 创建 Tkinter 窗口
        root = tk.Tk()
        root.title(title)

        # 获取颜色配置
        colors = AlertDialog.COLORS.get(level, AlertDialog.COLORS[AlertDialog.WARNING])

        # 设置窗口大小和位置（居中）
        window_width = 500
        window_height = 300 if details else 200
        screen_width = root.winfo_screenwidth()
        screen_height = root.winfo_screenheight()
        x = (screen_width - window_width) // 2
        y = (screen_height - window_height) // 2
        root.geometry(f"{window_width}x{window_height}+{x}+{y}")

        # 设置窗口属性
        root.configure(bg=colors["bg"])
        root.resizable(False, False)
        root.attributes('-topmost', True)  # 置顶显示

        # 创建主框架
        main_frame = tk.Frame(root, bg=colors["bg"], padx=20, pady=20)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # 标题栏（图标 + 标题）
        title_frame = tk.Frame(main_frame, bg=colors["bg"])
        title_frame.pack(fill=tk.X, pady=(0, 10))

        icon_label = tk.Label(
            title_frame,
            text=colors["icon"],
            font=("Arial", 24),
            bg=colors["bg"]
        )
        icon_label.pack(side=tk.LEFT, padx=(0, 10))

        title_label = tk.Label(
            title_frame,
            text=title,
            font=("Arial", 16, "bold"),
            fg=colors["fg"],
            bg=colors["bg"]
        )
        title_label.pack(side=tk.LEFT)

        # 消息内容
        message_label = tk.Label(
            main_frame,
            text=message,
            font=("Arial", 12),
            fg="#333333",
            bg=colors["bg"],
            wraplength=450,
            justify=tk.LEFT
        )
        message_label.pack(fill=tk.X, pady=(0, 10))

        # 详细信息（可选）
        if details:
            details_frame = tk.Frame(main_frame, bg="white", relief=tk.SUNKEN, bd=1)
            details_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))

            details_text = tk.Text(
                details_frame,
                font=("Courier", 10),
                bg="white",
                fg="#333333",
                wrap=tk.WORD,
                height=6,
                padx=10,
                pady=10
            )
            details_text.pack(fill=tk.BOTH, expand=True)
            details_text.insert(tk.END, details)
            details_text.config(state=tk.DISABLED)  # 只读

        # Button bar
        button_frame = tk.Frame(main_frame, bg=colors["bg"])
        button_frame.pack(fill=tk.X)

        def close_dialog():
            root.destroy()

        close_button = tk.Button(
            button_frame,
            text="OK",
            font=("Arial", 11, "bold"),
            bg=colors["fg"],
            fg="white",
            activebackground=colors["fg"],
            activeforeground="white",
            relief=tk.FLAT,
            padx=30,
            pady=8,
            command=close_dialog
        )
        close_button.pack(side=tk.RIGHT)

        # Auto-close (optional)
        if auto_close:
            countdown_label = tk.Label(
                button_frame,
                text=f"Auto-close in {auto_close} seconds",
                font=("Arial", 9),
                fg="#666666",
                bg=colors["bg"]
            )
            countdown_label.pack(side=tk.LEFT)

            def update_countdown(remaining):
                if remaining > 0:
                    countdown_label.config(text=f"Auto-close in {remaining} seconds")
                    root.after(1000, update_countdown, remaining - 1)
                else:
                    root.destroy()

            root.after(1000, update_countdown, auto_close - 1)
            root.after(auto_close * 1000, close_dialog)

        # 播放声音（可选）
        if sound:
            AlertDialog._play_sound(level)

        # 运行主循环
        root.mainloop()

    @staticmethod
    def _play_sound(level):
        """播放系统声音"""
        try:
            if level == AlertDialog.CRITICAL or level == AlertDialog.ERROR:
                # 播放错误声音
                subprocess.Popen(
                    ["paplay", "/usr/share/sounds/freedesktop/stereo/dialog-error.oga"],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
            elif level == AlertDialog.WARNING:
                # 播放警告声音
                subprocess.Popen(
                    ["paplay", "/usr/share/sounds/freedesktop/stereo/dialog-warning.oga"],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL
                )
        except Exception:
            pass  # 忽略声音播放失败

    # Convenience methods
    @staticmethod
    def info(title, message, details=None, sound=False, auto_close=None):
        """Show info dialog"""
        AlertDialog.show(title, message, details, AlertDialog.INFO, sound, auto_close)

    @staticmethod
    def warning(title, message, details=None, sound=False, auto_close=None):
        """Show warning dialog"""
        AlertDialog.show(title, message, details, AlertDialog.WARNING, sound, auto_close)

    @staticmethod
    def error(title, message, details=None, sound=True, auto_close=None):
        """Show error dialog (with sound by default)"""
        AlertDialog.show(title, message, details, AlertDialog.ERROR, sound, auto_close)

    @staticmethod
    def critical(title, message, details=None, sound=True, auto_close=None):
        """Show critical error dialog (with sound by default)"""
        AlertDialog.show(title, message, details, AlertDialog.CRITICAL, sound, auto_close)


# Test code
if __name__ == '__main__':
    import time

    print("Testing alert dialog system...")

    # Test 1: Info dialog
    print("\n1. Show info dialog (auto-close in 3 seconds)")
    AlertDialog.info(
        title="System Started",
        message="AUBO E5 linked execution system started successfully",
        details="Real robot: Connected\nGazebo: Started\nUnity: Connected",
        auto_close=3
    )
    time.sleep(4)

    # Test 2: Warning dialog
    print("\n2. Show warning dialog")
    AlertDialog.warning(
        title="Gazebo Convergence Slow",
        message="Gazebo virtual robot convergence time exceeded expected",
        details="Expected: 3.0 seconds\nActual: 5.2 seconds\nSuggestion: Check Gazebo RTF"
    )
    time.sleep(2)

    # Test 3: Error dialog (with sound)
    print("\n3. Show error dialog (with sound)")
    AlertDialog.error(
        title="Gazebo Convergence Failed",
        message="Gazebo virtual robot failed to converge within timeout",
        details="Max joint error: 0.05 rad (2.86°)\nTimeout: 8.0 seconds\nStatus: GOAL_TOLERANCE_VIOLATED",
        sound=True
    )
    time.sleep(2)

    # Test 4: Critical error dialog (with sound)
    print("\n4. Show critical error dialog (with sound)")
    AlertDialog.critical(
        title="Safety Check Failed",
        message="Large motion detected, collision risk!",
        details="Joint error: 0.8 rad (45.8°)\nThreshold: 0.5 rad (28.6°)\nAction: Stop immediately and check robot position",
        sound=True
    )

    print("\nTest completed!")
