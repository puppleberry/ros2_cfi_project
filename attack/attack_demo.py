#!/usr/bin/env python3
# attack_demo.py - ROS2 CFI 공격 시연 스크립트

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import sys
import subprocess
import os

class AttackDemo(Node):
    def __init__(self):
        super().__init__('attack_demo')
        self.publisher = self.create_publisher(String, 'vulnerable_topic', 10)
        
        print("\n" + "="*60)
        print("ROS2 CFI Attack Demonstration")
        print("="*60)
        
    def send_message(self, data, description=""):
        """메시지 전송 및 로깅"""
        if description:
            print(f"\n[*] {description}")
        
        msg = String()
        msg.data = data
        self.publisher.publish(msg)
        
        print(f"    → Sent: {data[:50]}{'...' if len(data) > 50 else ''}")
        time.sleep(0.5)  # 메시지 처리 대기
        
    def run_attack_sequence(self):
        """공격 시퀀스 실행"""
        
        # 1. 정상 동작 확인
        print("\n[Phase 1] Normal Operation Test")
        print("-" * 40)
        
        for i in range(3):
            self.send_message(f"Normal message {i+1}", f"Sending normal message {i+1}")
            time.sleep(0.5)
        
        # 상태 확인
        self.send_message("STATUS", "Checking node status")
        time.sleep(1)
        
        # 2. Buffer Overflow 시도
        print("\n[Phase 2] Buffer Overflow Attack")
        print("-" * 40)
        
        # 작은 오버플로우
        overflow_small = "OVERFLOW:" + "A" * 80
        self.send_message(overflow_small, "Small buffer overflow (80 bytes)")
        time.sleep(1)
        
        # 중간 오버플로우
        overflow_medium = "OVERFLOW:" + "B" * 200
        self.send_message(overflow_medium, "Medium buffer overflow (200 bytes)")
        time.sleep(1)
        
        # 상태 확인
        self.send_message("STATUS", "Checking corruption status")
        time.sleep(1)
        
        # 3. Subscription 변조 공격
        print("\n[Phase 3] Subscription Corruption Attack")
        print("-" * 40)
        
        self.send_message("OVERFLOW:CORRUPT_SUB", "Attempting to corrupt subscription object")
        time.sleep(1)
        
        # 4. CFI 트리거 시도
        print("\n[Phase 4] Triggering CFI Check")
        print("-" * 40)
        
        print("[*] Sending messages to trigger callback with corrupted subscription...")
        for i in range(3):
            self.send_message(f"Trigger {i+1}", f"Callback trigger {i+1} (should detect corruption)")
            time.sleep(0.5)
        
        print("\n[*] Attack sequence completed!")
        

def main():
    """메인 함수"""
    
    # CFI 활성화 여부 확인
    cfi_enabled = os.environ.get('LD_PRELOAD', '').find('libday6_final.so') != -1
    
    print("\n" + "="*60)
    print("ROS2 CFI Attack/Defense Demonstration")
    print("="*60)
    print(f"CFI Protection: {'ENABLED' if cfi_enabled else 'DISABLED'}")
    print("="*60)
    
    if not cfi_enabled:
        print("\n⚠️  WARNING: Running WITHOUT CFI protection!")
        print("   The subscription corruption will succeed.")
    else:
        print("\n✅ CFI protection is active.")
        print("   The attack should be detected and blocked.")
    
    input("\nPress Enter to start the attack sequence...")
    
    # ROS2 초기화 및 공격 실행
    rclpy.init()
    
    try:
        attacker = AttackDemo()
        
        # 약간의 대기 (노드 초기화)
        time.sleep(1)
        
        # 공격 시퀀스 실행
        attacker.run_attack_sequence()
        
        # 추가 대기
        time.sleep(2)
        
    except KeyboardInterrupt:
        print("\n[!] Attack interrupted by user")
    except Exception as e:
        print(f"\n[!] Error during attack: {e}")
    finally:
        rclpy.shutdown()
    
    print("\n" + "="*60)
    print("Demonstration completed")
    print("="*60)
    

if __name__ == '__main__':
    main()
