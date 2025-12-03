#!/usr/bin/env python3
import glob
import os
import sys
import carla
import random
import time

# ==============================================================================
# CARLA 모듈 경로 설정
# ==============================================================================
try:
    sys.path.append(glob.glob('../../../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

def main():
    # ==============================================================================
    # [사용자 설정] 장애물 종류별 스폰 위치 설정 (모든 유형 포함)
    # 원하는 위치의 번호(ID)를 리스트에 적으세요. (비워두면 스폰 안 함)
    # ==============================================================================
    
    # 1. 차량 (Vehicle) - 주차된 차
    target_vehicle_indices = [120, 200]
    
    # 2. 고깔 (Cone) - 공사 구간 재현
    target_cone_indices = [50, 51, 52]
    
    # 3. 안전 펜스 (Barrier) - 도로 차단용
    target_barrier_indices = [] 
    
    # 4. 자전거/오토바이 (Cyclist) - 이륜차
    target_cyclist_indices = [85]

    # 5. 박스 (Box) - 낙하물 (종이박스, 나무상자 등)
    target_box_indices = [82]

    # 6. 드럼통 (Barrel) - 충격 흡수통, 공사 자재
    target_barrel_indices = []

    # 7. 쓰레기통/자판기 (Urban Object) - 도심 장애물
    target_trash_indices = []

    # 8. 타이어 (Tire) - 타이어 낙하물
    target_tire_indices = []

    # 9. 공사 표지판 (Warning Sign) - 공사중/주의 표지판
    target_sign_indices = []

    # ==============================================================================

    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0)
    world = client.get_world()
    bp_lib = world.get_blueprint_library()
    spawn_points = world.get_map().get_spawn_points()

    actor_list = []

    # 장애물 생성 함수
    def spawn_obstacles(indices, filter_pattern, type_name):
        if not indices:
            return

        print(f"Spawning {type_name} ({filter_pattern}) at: {indices}")
        
        for idx in indices:
            if idx >= len(spawn_points):
                print(f"  [Warning] Index {idx} is out of range. Skipping.")
                continue
            
            transform = spawn_points[idx]
            
            # 블루프린트 검색
            blueprints = bp_lib.filter(filter_pattern)
            if not blueprints:
                print(f"  [Error] No blueprints found for {filter_pattern}")
                continue

            # 차량 필터링 (4륜차만)
            if type_name == "Vehicle":
                blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
            
            bp = random.choice(blueprints)

            # 차량 색상 랜덤 설정
            if bp.has_attribute('color'):
                color = random.choice(bp.get_attribute('color').recommended_values)
                bp.set_attribute('color', color)

            # 스폰 시도
            actor = world.try_spawn_actor(bp, transform)
            
            if actor:
                # 물리 엔진 및 고정 설정
                if type_name in ["Vehicle", "Cyclist"]:
                    actor.set_autopilot(False)
                    actor.set_simulate_physics(True)
                    control = carla.VehicleControl()
                    control.hand_brake = True
                    actor.apply_control(control)
                else:
                    # 정적 물체는 물리 엔진 켜두면 자연스럽게 바닥에 놓임
                    actor.set_simulate_physics(True)
                
                actor_list.append(actor)
                print(f"  -> Spawned {bp.id} at Index {idx}")
            else:
                print(f"  -> Failed to spawn at Index {idx} (Collision?)")

    try:
        # 모든 종류별 장애물 생성 실행
        spawn_obstacles(target_vehicle_indices, 'vehicle.*', "Vehicle")
        spawn_obstacles(target_cyclist_indices, 'vehicle.bh.crossbike', "Cyclist")
        
        # 정적 장애물 (Props)
        spawn_obstacles(target_cone_indices, 'static.prop.constructioncone', "Cone")
        spawn_obstacles(target_barrier_indices, 'static.prop.streetbarrier', "Barrier")
        
        # 추가된 장애물들
        spawn_obstacles(target_box_indices, 'static.prop.box*', "Box") # box01, box02, creasedbox 등
        spawn_obstacles(target_barrel_indices, 'static.prop.barrel', "Barrel")
        spawn_obstacles(target_trash_indices, 'static.prop.trashcan*', "TrashCan") # trashcan01 ~ 05
        spawn_obstacles(target_tire_indices, 'static.prop.tire', "Tire")
        spawn_obstacles(target_sign_indices, 'static.prop.warning*', "Sign") # warningconstruction, accident

        print(f"\nSuccessfully spawned total {len(actor_list)} obstacles.")
        print("Press Ctrl+C to remove obstacles and exit.")

        # 무한 대기
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nRemoving obstacles...")
    finally:
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])
        print("Done.")

if __name__ == '__main__':
    main()