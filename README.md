# zeus_pnp

## 환경 구축
PC
    
    conda env create --file realsense.yaml
    # pip install pyrealsense2
    # pip install opencv-python==4.5.3.56

ZEUS 와 체스보드 첫번째 코너와의 실측 거리 입력

    # librealsense/wrappers/python/examples/box_dimensioner_multicam/measurement_task.py
    default_x = [실측거리] 
    default_y = [실측거리]
    default_z = [실측거리]
    
    
PC 실행 

    cd librealsense/wrappers/python/examples/box_dimensioner_multicam
    python box_dimensioner_multicam_demo.py

ZEUS 실행 코드

    python jh_cap_zeus_move.py
    
    

## License
This project is licensed under the [Apache License, Version 2.0](LICENSE).
Copyright 2018 Intel Corporation

