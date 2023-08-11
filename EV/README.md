msg:morai에서 제공하는 메시지들을 저장
path: SOO_path_maker.py에서 제작하는 path를 저장하는 폴더
script:코드 저장공간
script-trash : 사용하다가 필요 없어진 코드들
src: 자료 폴더

script 폴더

Camera_Human_detect.py 
카메라에서 사람을 Cascade방식으로 인식하는 코드
카메라에서 사람을 인식하면 is_human = True로 바꾸고
is_human이 True이면 사람이 인식되었다고 출력한다.
주의사항으로 harrcascade_fullbody.xml 파일이 있는지 확인해야한다.

SOO_path_maker.py
좌표값을 저장하는 파일 
이때 저장되는 값은 Offset기준이다.

get_speed.py
IMU센서에서 Speed값을 추출해낸다
현재 구현이 잘 안되어 있음

lane_drive.py
자이트론 라인트래킹 함수를 베이스로 제작한 파일

traffic_light_roi.py
신호등 값을 읽어옴


src 폴더

haarcascade_fullbody.xml
사람인식용 cascade파일 opencv제공

setting.json
morai시뮬레이터용 초기 셋팅

t_trackz_track.json
morai시뮬레이터용 꼬깔콘 셋팅

