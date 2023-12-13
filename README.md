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
    

## 실행

PC 실행 

    cd librealsense/wrappers/python/examples/box_dimensioner_multicam
    python box_dimensioner_multicam_demo.py


### 실행화면 

![Screenshot from 2023-12-08 05-28-06](https://github.com/DDochi98/zeus_pnp/assets/70254727/d9744e37-d414-4a6f-8996-0b054eaf4057)

ZEUS 실행 코드

    python jh_cap_zeus_move.py



    

## License
This project is licensed under the [Apache License, Version 2.0](LICENSE).
Copyright 2018 Intel Corporation

