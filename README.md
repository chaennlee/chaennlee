[![Top Langs](https://github-readme-stats.vercel.app/api/top-langs/?username=chaennlee&layout=compact)](https://github.com/chaennlee/github-readme-stats)
  
  👋 @chaennlee

  
  👀  🌱 working working working

  
  📫 Email : eun020509@naver.com




Hyundai Robot Job File; { version: 2.0, mech_type: "934(HDF7-9AP)", total_axis: 10, aux_axis: 4 }
	_mw80=-1 #잡 에러처리 초기화
	_mw74=40 #잡 번호 업데이트 - 대차정렬
	_mw76=400 #대차정렬 진행단계 - 변수 선언 및 설정 쓰기
	gosub *setting
	contpath 2
	var idx=0.00 #반복문 인덱스
	var currentPose #현재 포즈 기록용
	var tgt_dist, ang ########################## new 거리, 거리변환각도 
	var temp
	endless zero,axis=[9,10] ########################## new 축 설정

	def lin2deg,dist,jidx ########################## new 함수 
       var angrad, angdeg, diameter, turn_cnt, tgt_ang
       diameter=122 # 바퀴 직경 122mm, 현장
       angrad = dist/(3.1415*diameter)
       angdeg = rad2deg(angrad)
       turn_cnt =floor(angdeg/360) # endless 회전 카운트
       tgt_ang = angdeg - turn_cnt*360 # 멀티턴 회전 후 잔여 이동 각
       if turn_cnt>0
       endless turn,axis=jidx,count=turn_cnt
       endif
       return tgt_ang

	var shiftTurn=0.00 #조향 이동량
	var shiftDrive=0.00 #주행 이동량
	var shiftAngle=0.00 #주행 각도
	var nowErr=0.00 #PID 제어용 현재오차
	var preErr=0.00 #PID 제어용 직전오차
	var sumErr=0.00 #PID 제어용 누적오차
	var resultPID=0.00 #PID 제어량
	var leftDrive=0.00 #PID 좌측 이동량
	var rightDrive=0.00 #PID 우측 이동량
	_mw76=401 #대차정렬 진행단계 - 잡 시작
	#시작 좌표 = [136.90,-4.20,512.80,179.90,-35.00,-179.90]
	#시작 축각도 = [0.00,117.90,-10.00,0.00,-97.90,0.00]
	#시작 대차각도 = [0.00,0.00,0.00,0.00] ############################################# 싹 바뀌어야 할 것 같은데.. 이 '좌표' '축각도' '대차각도' 
	_mw76=402 #대차정렬 진행단계 - 정렬 생략 여부 판단
	if di86 then *end_align #정렬 종료
	*dir_align1
	if (di92 and di82) or (di93 and di83) then *center_align1 #방향 정렬 생략
	if (not di82) and (not di83) then *drive_align #방향/중앙 정렬 생략
	_mw76=403 #대차정렬 진행단계 - 방향 정렬 시작
	currentPose=cpo("joint","cur")
	currentPose.j7=90
	currentPose.j8=0
S1	move P,tg=currentPose,spd=_mw346%,accu=0,tool=0 #선회 조향
	for idx=0 to 4 step 1
	if (di92 and di82) or (di93 and di83)
	_mw76=404 #대차정렬 진행단계 - 방향 정렬 완료
	break
	endif
	if di82 #좌측 기준 정렬
	shiftTurn=atan2(_mf636, _mf620) * _mf628 / 2
	elseif di83 #우측 기준 정렬
	shiftTurn=-atan2(_mf640, _mf620) * _mf628 / 2
	else #벽 없음
	goto *drive_align #방향/중앙 정렬 생략
	endif
	currentPose=cpo("joint","cur")
	call lin2deg,shiftTurn,9
	currentPose.j9=currentPose.j9 - result()
	call lin2deg,shiftTurn,10
	currentPose.j10=currentPose.j10 + result()
S2	move P,tg=currentPose,spd=_mw344%,accu=0,tool=0 #선회 주행
	next
	*center_align1
	if (not di82) and (not di83) then *drive_align #방향/중앙 정렬 생략
	_mw76=405 #대차정렬 진행단계 - 중앙 정렬 시작
	currentPose=cpo("joint","cur")
	if di82 #좌측 기준 정렬
	if di94 then *PID_align #중앙 정렬 생략
	if di83 #센서 기반 셀폭 계산
	_mf660=(_mf652 * cos(atan2(_mf636, _mf620)) - _mf624) / 2
	else #수치 기반 셀폭 계산
	_mf660=(840.00 * cos(atan2(_mf636, _mf620)) - _mf624) / 2
	endif
	shiftAngle=atan2(_mf680, min(350, _mf700))
	if sin(shiftAngle) == 0 then *PID_align #중앙 정렬 생략
	if _mf660 < _mf604
	currentPose.j7=min(abs(rad2deg(shiftAngle)), _mf632)
	currentPose.j8=min(abs(rad2deg(shiftAngle)), _mf632)
	else
	currentPose.j7=-min(abs(rad2deg(shiftAngle)), _mf632)
	currentPose.j8=-min(abs(rad2deg(shiftAngle)), _mf632)
	endif
	shiftDrive=min(abs(_mf680 / sin(shiftAngle)), 350)
	elseif di83 #우측 기준 정렬, 수치 기반 셀폭 계산
	if di95 then *PID_align #중앙 정렬 생략
	_mf660=(840.00 * cos(atan2(_mf640, _mf620)) - _mf624) / 2
	shiftAngle=atan2(_mf684, min(350, _mf700))
	if sin(shiftAngle) == 0 then *PID_align #중앙 정렬 생략
	if _mf660 > _mf612
	currentPose.j7=min(abs(rad2deg(shiftAngle)), _mf632)
	currentPose.j8=min(abs(rad2deg(shiftAngle)), _mf632)
	else
	currentPose.j7=-min(abs(rad2deg(shiftAngle)), _mf632)
	currentPose.j8=-min(abs(rad2deg(shiftAngle)), _mf632)
	endif
	shiftDrive=min(abs(_mf684 / sin(shiftAngle)), 350)
	else #벽 없음
	goto *drive_align #방향/중앙 정렬 생략
	endif
S3	move P,tg=currentPose,spd=_mw346%,accu=0,tool=0 #사선 조향
	currentPose=cpo("joint","cur") #add 
	call lin2deg,shiftDrive,9
	currentPose.j9=currentPose.j9 + result()
	call lin2deg,shiftDrive,10
	currentPose.j10=currentPose.j10 + result()
S4	move P,tg=currentPose,spd=_mw342%,accu=0,tool=0 #사선 주행
	_mw76=406 #대차정렬 진행단계 - 중앙 정렬 완료
	*PID_align
	if not di98 then *drive_align #PID/방향/중앙 정렬 생략
	_mw76=407 #대차정렬 진행단계 - PID 정렬 시작
	currentPose=cpo("joint","cur")
	currentPose.j7=0
	currentPose.j8=0
S5	move P,tg=currentPose,spd=_mw346%,accu=0,tool=0 #주행 조향
	if di82 #좌측 기준 주행
	if di83 #센서 기반 셀폭 계산
	_mf660=(_mf652 * cos(atan2(_mf636, _mf620)) - _mf624) / 2
	else #수치 기반 셀폭 계산
	_mf660=(840.00 * cos(atan2(_mf636, _mf620)) - _mf624) / 2
	endif
	elseif di83 #우측 기준 주행, 수치 기반 셀폭 계산
	_mf660=(840.00 * cos(atan2(_mf640, _mf620)) - _mf624) / 2
	else #벽 없음
	goto *drive_align #PID/방향/중앙 정렬 생략
	endif
	for idx=0 to 2000 step 1
	if not di98
	_mw76=408 #대차정렬 진행단계 - PID 정렬 완료
	break
	endif
	nowErr=_mf332
	sumErr=sumErr + nowErr
	resultPID=2 + _mf364 * nowErr + _mf368 * sumErr / 0.1 + _mf372 * (nowErr - preErr) * 0.1
	resultPID=max(min(nowErr, 14.9), -14.9)
	preErr=nowErr
	leftDrive=2 * asin((15 + resultPID) / (2 * _mf620)) * _mf628
	rightDrive=2 * asin((15 - resultPID) / (2 * _mf620)) * _mf628
	if di80 #PID 저속
	temp=leftDrive + _mw322
	call lin2deg,temp,9
	currentPose.j9=currentPose.j9 + result()
	temp=rightDrive + _mw322
	call lin2deg,temp,10
	currentPose.j10=currentPose.j10 + result()
	else #PID 고속
	temp=leftDrive + _mw348
	call lin2deg,temp,9
	currentPose.j9=currentPose.j9 + result()
	temp=rightDrive + _mw348
	call lin2deg,temp,10
	currentPose.j10=currentPose.j10 + result()
	endif
S6	move P,tg=currentPose,spd=_mw340%,accu=2,tool=0 #PID 주행
	next
	*dir_align2
	if (di92 and di82) or (di93 and di83) then *center_align2 #중간 방향 정렬 생략
	if (not di82) and (not di83) then *drive_align #방향/중앙 정렬 생략
	_mw76=409 #대차정렬 진행단계 - 방향 정렬 시작
	currentPose=cpo("joint","cur")
	currentPose.j7=90
	currentPose.j8=0
S7	move P,tg=currentPose,spd=_mw346%,accu=0,tool=0 #선회 조향
	for idx=0 to 4 step 1
	if (di92 and di82) or (di93 and di83)
	_mw76=410 #대차정렬 진행단계 - 방향 정렬 완료
	break
	endif
	if di82 #좌측 기준 정렬
	shiftTurn=atan2(_mf636, _mf620) * _mf628 / 2
	elseif di83 #우측 기준 정렬
	shiftTurn=-atan2(_mf640, _mf620) * _mf628 / 2
	else #벽 없음
	goto *drive_align #방향/중앙 정렬 생략
	endif
	currentPose=cpo("joint","cur")
	call lin2deg,shiftTurn,9
	currentPose.j9=currentPose.j9 - result()
	call lin2deg,shiftTurn,10
	currentPose.j10=currentPose.j10 + result()
S8	move P,tg=currentPose,spd=_mw344%,accu=0,tool=0 #선회 주행
	next
	*center_align2
	if (not di82) and (not di83) then *drive_align #방향/중앙 정렬 생략
	_mw76=411 #대차정렬 진행단계 - 중앙 정렬 시작
	currentPose=cpo("joint","cur")
	if di82 #좌측 기준 정렬
	if di94 then *dir_align3 #중앙 정렬 생략
	if di83 #센서 기반 셀폭 계산
	_mf660=(_mf652 * cos(atan2(_mf636, _mf620)) - _mf624) / 2
	else #수치 기반 셀폭 계산
	_mf660=(840.00 * cos(atan2(_mf636, _mf620)) - _mf624) / 2
	endif
	shiftAngle=atan2(_mf680, min(350, _mf700))
	if sin(shiftAngle) == 0 then *dir_align3 #중앙 정렬 생략
	if _mf660 < _mf604
	currentPose.j7=min(abs(rad2deg(shiftAngle)), _mf632)
	currentPose.j8=min(abs(rad2deg(shiftAngle)), _mf632)
	else
	currentPose.j7=-min(abs(rad2deg(shiftAngle)), _mf632)
	currentPose.j8=-min(abs(rad2deg(shiftAngle)), _mf632)
	endif
	shiftDrive=min(abs(_mf680 / sin(shiftAngle)), 350)
	elseif di83 #우측 기준 정렬, 수치 기반 셀폭 계산
	if di95 then *dir_align3 #중앙 정렬 생략
	_mf660=(840.00 * cos(atan2(_mf640, _mf620)) - _mf624) / 2
	shiftAngle=atan2(_mf684, min(350, _mf700))
	if sin(shiftAngle) == 0 then *dir_align3 #중앙 정렬 생략
	if _mf660 > _mf612
	currentPose.j7=min(abs(rad2deg(shiftAngle)), _mf632)
	currentPose.j8=min(abs(rad2deg(shiftAngle)), _mf632)
	else
	currentPose.j7=-min(abs(rad2deg(shiftAngle)), _mf632)
	currentPose.j8=-min(abs(rad2deg(shiftAngle)), _mf632)
	endif
	shiftDrive=min(abs(_mf684 / sin(shiftAngle)), 350)
	else #벽 없음
	goto *drive_align #방향/중앙 정렬 생략
	endif
S9	move P,tg=currentPose,spd=_mw346%,accu=0,tool=0 #사선 조향
	currentPose=cpo("joint","cur") #add 
	call lin2deg,shiftDrive,9
	currentPose.j9=currentPose.j9 + result()
	call lin2deg,shiftDrive,10
	currentPose.j10=currentPose.j10 + result()
S10	move P,tg=currentPose,spd=_mw342%,accu=0,tool=0 #사선 주행
	_mw76=412 #대차정렬 진행단계 - 중앙 정렬 완료
	*dir_align3
	if (di92 and di82) or (di93 and di83) then *drive_align #최종 방향 정렬 생략
	if (not di82) and (not di83) then *drive_align #방향 정렬 생략
	_mw76=413 #대차정렬 진행단계 - 방향 정렬 시작
	currentPose=cpo("joint","cur")
	currentPose.j7=90
	currentPose.j8=0
S11	move P,tg=currentPose,spd=_mw346%,accu=0,tool=0 #선회 조향
	for idx=0 to 4 step 1
	if (di92 and di82) or (di93 and di83)
	_mw76=414 #대차정렬 진행단계 - 방향 정렬 완료
	break
	endif
	if di82 #좌측 기준 정렬
	shiftTurn=atan2(_mf636, _mf620) * _mf628 / 2
	elseif di83 #우측 기준 정렬
	shiftTurn=-atan2(_mf640, _mf620) * _mf628 / 2
	else #벽 없음
	goto *drive_align #방향/중앙 정렬 생략
	endif
	currentPose=cpo("joint","cur")
	call lin2deg,shiftTurn,9
	currentPose.j9=currentPose.j9 - result()
	call lin2deg,shiftTurn,10
	currentPose.j10=currentPose.j10 + result()
S12	move P,tg=currentPose,spd=_mw344%,accu=0,tool=0 #선회 주행
	next
	*drive_align
	if di86 then *end_align #정렬 종료
	_mw76=415 #대차정렬 진행단계 - 최종 거리 정렬 시작
	currentPose=cpo("joint","cur")
	currentPose.j7=0
	currentPose.j8=0
S13	move P,tg=currentPose,spd=_mw346%,accu=0,tool=0 #주행 조향
	if di85 #전방 가까움
	_mw76=416 #대차정렬 진행단계 - 이격
	for idx=0 to 2000 step 1
	if di86 or di87 or di88
	_mw76=417 #대차정렬 진행단계 - 최종 이격 완료
	break
	endif
	currentPose=cpo("joint","cur")
	call lin2deg,_mw350,9
	currentPose.j9=currentPose.j9 - result()
	call lin2deg,_mw350,10
	currentPose.j10=currentPose.j10 - result()
S14	move P,tg=currentPose,spd=_mw340%,accu=2,tool=0 #이격 주행
	next
	elseif di87 #전방 멈
	_mw76=418 #대차정렬 진행단계 - 접근
	for idx=0 to 2000 step 1
	if di85 or di86
	_mw76=419 #대차정렬 진행단계 - 최종 접근 완료
	break
	endif
	currentPose=cpo("joint","cur")
	call lin2deg,_mw350,9
	currentPose.j9=currentPose.j9 + result()
	call lin2deg,_mw350,10
	currentPose.j10=currentPose.j10 + result()
S15	move P,tg=currentPose,spd=_mw340%,accu=2,tool=0 #일반 주행
	next
	endif
	*end_align
	currentPose=cpo("joint","cur")
	currentPose.j7=0
	currentPose.j8=0
S16	move P,tg=currentPose,spd=_mw346%,accu=0,tool=0 #조향 원위치
	_mw76=497 #대차정렬 진행단계 - 조향 원위치 완료
	_mw76=498 #대차정렬 진행단계 - 설정 쓰기
	gosub *setting
	_mw76=499 #대차정렬 진행단계 - 잡 완료
	_mw74=0 #잡 번호 초기화
end
	*setting #설정
	contpath 0
	_spd_rate=100 #재생속도 설정
	_tool[0]=Shift(_mw194/10,_mw196/10,_mw198/10,_mw200/10,_mw202/10,_mw204/10,"tool") #툴 데이터 설정
	var ucrdAssign=mkucs(1,Pose(_mw208/10,_mw210/10,_mw212/10,_mw214/10,_mw216/10,_mw218/10,0,0,0,0,"base")) #사용자 좌표계 데이터 설정
	seltool 0,robot #툴 선택
	selucrd 1 #사용자 좌표계 선택
	retsub



Hyundai Robot Job File; { version: 2.0, mech_type: "934(HDF7-9AP)", total_axis: 10, aux_axis: 4 }
	_mw80=-1 #잡 에러처리 초기화
	_mw74=40 #잡 번호 업데이트 - 대차정렬
	_mw76=400 #대차정렬 진행단계 - 변수 선언 및 설정 쓰기
	gosub *setting
	contpath 2
	var idx=0.00 #반복문 인덱스
	var currentPose #현재 포즈 기록용
	var tgt_dist, ang ########################## new 거리, 거리변환각도 
	var temp
	endless zero,axis=[9,10] ########################## new 축 설정

	def lin2deg,dist,jidx ########################## new 함수 
       var angrad, angdeg, diameter, turn_cnt, tgt_ang
       diameter=122 # 바퀴 직경 122mm, 현장
       angrad = dist/(3.1415*diameter)
       angdeg = rad2deg(angrad)
       return angdeg 

	var shiftTurn=0.00 #조향 이동량
	var shiftDrive=0.00 #주행 이동량
	var shiftAngle=0.00 #주행 각도
	var nowErr=0.00 #PID 제어용 현재오차
	var preErr=0.00 #PID 제어용 직전오차
	var sumErr=0.00 #PID 제어용 누적오차
	var resultPID=0.00 #PID 제어량
	var leftDrive=0.00 #PID 좌측 이동량
	var rightDrive=0.00 #PID 우측 이동량
	_mw76=401 #대차정렬 진행단계 - 잡 시작
	#시작 좌표 = [136.90,-4.20,512.80,179.90,-35.00,-179.90]
	#시작 축각도 = [0.00,117.90,-10.00,0.00,-97.90,0.00]
	#시작 대차각도 = [0.00,0.00,0.00,0.00] ############################################# 싹 바뀌어야 할 것 같은데.. 이 '좌표' '축각도' '대차각도' 
	_mw76=402 #대차정렬 진행단계 - 정렬 생략 여부 판단
	if di86 then *end_align #정렬 종료
	*dir_align1
	if (di92 and di82) or (di93 and di83) then *center_align1 #방향 정렬 생략
	if (not di82) and (not di83) then *drive_align #방향/중앙 정렬 생략
	_mw76=403 #대차정렬 진행단계 - 방향 정렬 시작
	currentPose=cpo("joint","cur")
	currentPose.j7=90
	currentPose.j8=0
S1	move P,tg=currentPose,spd=_mw346%,accu=0,tool=0 #선회 조향
	for idx=0 to 4 step 1
	if (di92 and di82) or (di93 and di83)
	_mw76=404 #대차정렬 진행단계 - 방향 정렬 완료
	break
	endif
	if di82 #좌측 기준 정렬
	shiftTurn=atan2(_mf636, _mf620) * _mf628 / 2
	elseif di83 #우측 기준 정렬
	shiftTurn=-atan2(_mf640, _mf620) * _mf628 / 2
	else #벽 없음
	goto *drive_align #방향/중앙 정렬 생략
	endif
	currentPose=cpo("joint","cur")
	call lin2deg,shiftTurn,9
	currentPose.j9=currentPose.j9 - result()
	call lin2deg,shiftTurn,10
	currentPose.j10=currentPose.j10 + result()
S2	move P,tg=currentPose,spd=_mw344%,accu=0,tool=0 #선회 주행
	next
	*center_align1
	if (not di82) and (not di83) then *drive_align #방향/중앙 정렬 생략
	_mw76=405 #대차정렬 진행단계 - 중앙 정렬 시작
	currentPose=cpo("joint","cur")
	if di82 #좌측 기준 정렬
	if di94 then *PID_align #중앙 정렬 생략
	if di83 #센서 기반 셀폭 계산
	_mf660=(_mf652 * cos(atan2(_mf636, _mf620)) - _mf624) / 2
	else #수치 기반 셀폭 계산
	_mf660=(840.00 * cos(atan2(_mf636, _mf620)) - _mf624) / 2
	endif
	shiftAngle=atan2(_mf680, min(350, _mf700))
	if sin(shiftAngle) == 0 then *PID_align #중앙 정렬 생략
	if _mf660 < _mf604
	currentPose.j7=min(abs(rad2deg(shiftAngle)), _mf632)
	currentPose.j8=min(abs(rad2deg(shiftAngle)), _mf632)
	else
	currentPose.j7=-min(abs(rad2deg(shiftAngle)), _mf632)
	currentPose.j8=-min(abs(rad2deg(shiftAngle)), _mf632)
	endif
	shiftDrive=min(abs(_mf680 / sin(shiftAngle)), 350)
	elseif di83 #우측 기준 정렬, 수치 기반 셀폭 계산
	if di95 then *PID_align #중앙 정렬 생략
	_mf660=(840.00 * cos(atan2(_mf640, _mf620)) - _mf624) / 2
	shiftAngle=atan2(_mf684, min(350, _mf700))
	if sin(shiftAngle) == 0 then *PID_align #중앙 정렬 생략
	if _mf660 > _mf612
	currentPose.j7=min(abs(rad2deg(shiftAngle)), _mf632)
	currentPose.j8=min(abs(rad2deg(shiftAngle)), _mf632)
	else
	currentPose.j7=-min(abs(rad2deg(shiftAngle)), _mf632)
	currentPose.j8=-min(abs(rad2deg(shiftAngle)), _mf632)
	endif
	shiftDrive=min(abs(_mf684 / sin(shiftAngle)), 350)
	else #벽 없음
	goto *drive_align #방향/중앙 정렬 생략
	endif
S3	move P,tg=currentPose,spd=_mw346%,accu=0,tool=0 #사선 조향
	currentPose=cpo("joint","cur") #add 
	call lin2deg,shiftDrive,9
	currentPose.j9=currentPose.j9 + result()
	call lin2deg,shiftDrive,10
	currentPose.j10=currentPose.j10 + result()
S4	move P,tg=currentPose,spd=_mw342%,accu=0,tool=0 #사선 주행
	_mw76=406 #대차정렬 진행단계 - 중앙 정렬 완료
	*PID_align
	if not di98 then *drive_align #PID/방향/중앙 정렬 생략
	_mw76=407 #대차정렬 진행단계 - PID 정렬 시작
	currentPose=cpo("joint","cur")
	currentPose.j7=0
	currentPose.j8=0
S5	move P,tg=currentPose,spd=_mw346%,accu=0,tool=0 #주행 조향
	if di82 #좌측 기준 주행
	if di83 #센서 기반 셀폭 계산
	_mf660=(_mf652 * cos(atan2(_mf636, _mf620)) - _mf624) / 2
	else #수치 기반 셀폭 계산
	_mf660=(840.00 * cos(atan2(_mf636, _mf620)) - _mf624) / 2
	endif
	elseif di83 #우측 기준 주행, 수치 기반 셀폭 계산
	_mf660=(840.00 * cos(atan2(_mf640, _mf620)) - _mf624) / 2
	else #벽 없음
	goto *drive_align #PID/방향/중앙 정렬 생략
	endif
	for idx=0 to 2000 step 1
	if not di98
	_mw76=408 #대차정렬 진행단계 - PID 정렬 완료
	break
	endif
	nowErr=_mf332
	sumErr=sumErr + nowErr
	resultPID=2 + _mf364 * nowErr + _mf368 * sumErr / 0.1 + _mf372 * (nowErr - preErr) * 0.1
	resultPID=max(min(nowErr, 14.9), -14.9)
	preErr=nowErr
	leftDrive=2 * asin((15 + resultPID) / (2 * _mf620)) * _mf628
	rightDrive=2 * asin((15 - resultPID) / (2 * _mf620)) * _mf628
	if di80 #PID 저속
	temp=leftDrive + _mw322
	call lin2deg,temp,9
	currentPose.j9=currentPose.j9 + result()
	temp=rightDrive + _mw322
	call lin2deg,temp,10
	currentPose.j10=currentPose.j10 + result()
	else #PID 고속
	temp=leftDrive + _mw348
	call lin2deg,temp,9
	currentPose.j9=currentPose.j9 + result()
	temp=rightDrive + _mw348
	call lin2deg,temp,10
	currentPose.j10=currentPose.j10 + result()
	endif
S6	move P,tg=currentPose,spd=_mw340%,accu=2,tool=0 #PID 주행
	next
	*dir_align2
	if (di92 and di82) or (di93 and di83) then *center_align2 #중간 방향 정렬 생략
	if (not di82) and (not di83) then *drive_align #방향/중앙 정렬 생략
	_mw76=409 #대차정렬 진행단계 - 방향 정렬 시작
	currentPose=cpo("joint","cur")
	currentPose.j7=90
	currentPose.j8=0
S7	move P,tg=currentPose,spd=_mw346%,accu=0,tool=0 #선회 조향
	for idx=0 to 4 step 1
	if (di92 and di82) or (di93 and di83)
	_mw76=410 #대차정렬 진행단계 - 방향 정렬 완료
	break
	endif
	if di82 #좌측 기준 정렬
	shiftTurn=atan2(_mf636, _mf620) * _mf628 / 2
	elseif di83 #우측 기준 정렬
	shiftTurn=-atan2(_mf640, _mf620) * _mf628 / 2
	else #벽 없음
	goto *drive_align #방향/중앙 정렬 생략
	endif
	currentPose=cpo("joint","cur")
	call lin2deg,shiftTurn,9
	currentPose.j9=currentPose.j9 - result()
	call lin2deg,shiftTurn,10
	currentPose.j10=currentPose.j10 + result()
S8	move P,tg=currentPose,spd=_mw344%,accu=0,tool=0 #선회 주행
	next
	*center_align2
	if (not di82) and (not di83) then *drive_align #방향/중앙 정렬 생략
	_mw76=411 #대차정렬 진행단계 - 중앙 정렬 시작
	currentPose=cpo("joint","cur")
	if di82 #좌측 기준 정렬
	if di94 then *dir_align3 #중앙 정렬 생략
	if di83 #센서 기반 셀폭 계산
	_mf660=(_mf652 * cos(atan2(_mf636, _mf620)) - _mf624) / 2
	else #수치 기반 셀폭 계산
	_mf660=(840.00 * cos(atan2(_mf636, _mf620)) - _mf624) / 2
	endif
	shiftAngle=atan2(_mf680, min(350, _mf700))
	if sin(shiftAngle) == 0 then *dir_align3 #중앙 정렬 생략
	if _mf660 < _mf604
	currentPose.j7=min(abs(rad2deg(shiftAngle)), _mf632)
	currentPose.j8=min(abs(rad2deg(shiftAngle)), _mf632)
	else
	currentPose.j7=-min(abs(rad2deg(shiftAngle)), _mf632)
	currentPose.j8=-min(abs(rad2deg(shiftAngle)), _mf632)
	endif
	shiftDrive=min(abs(_mf680 / sin(shiftAngle)), 350)
	elseif di83 #우측 기준 정렬, 수치 기반 셀폭 계산
	if di95 then *dir_align3 #중앙 정렬 생략
	_mf660=(840.00 * cos(atan2(_mf640, _mf620)) - _mf624) / 2
	shiftAngle=atan2(_mf684, min(350, _mf700))
	if sin(shiftAngle) == 0 then *dir_align3 #중앙 정렬 생략
	if _mf660 > _mf612
	currentPose.j7=min(abs(rad2deg(shiftAngle)), _mf632)
	currentPose.j8=min(abs(rad2deg(shiftAngle)), _mf632)
	else
	currentPose.j7=-min(abs(rad2deg(shiftAngle)), _mf632)
	currentPose.j8=-min(abs(rad2deg(shiftAngle)), _mf632)
	endif
	shiftDrive=min(abs(_mf684 / sin(shiftAngle)), 350)
	else #벽 없음
	goto *drive_align #방향/중앙 정렬 생략
	endif
S9	move P,tg=currentPose,spd=_mw346%,accu=0,tool=0 #사선 조향
	currentPose=cpo("joint","cur") #add 
	call lin2deg,shiftDrive,9
	currentPose.j9=currentPose.j9 + result()
	call lin2deg,shiftDrive,10
	currentPose.j10=currentPose.j10 + result()
S10	move P,tg=currentPose,spd=_mw342%,accu=0,tool=0 #사선 주행
	_mw76=412 #대차정렬 진행단계 - 중앙 정렬 완료
	*dir_align3
	if (di92 and di82) or (di93 and di83) then *drive_align #최종 방향 정렬 생략
	if (not di82) and (not di83) then *drive_align #방향 정렬 생략
	_mw76=413 #대차정렬 진행단계 - 방향 정렬 시작
	currentPose=cpo("joint","cur")
	currentPose.j7=90
	currentPose.j8=0
S11	move P,tg=currentPose,spd=_mw346%,accu=0,tool=0 #선회 조향
	for idx=0 to 4 step 1
	if (di92 and di82) or (di93 and di83)
	_mw76=414 #대차정렬 진행단계 - 방향 정렬 완료
	break
	endif
	if di82 #좌측 기준 정렬
	shiftTurn=atan2(_mf636, _mf620) * _mf628 / 2
	elseif di83 #우측 기준 정렬
	shiftTurn=-atan2(_mf640, _mf620) * _mf628 / 2
	else #벽 없음
	goto *drive_align #방향/중앙 정렬 생략
	endif
	currentPose=cpo("joint","cur")
	call lin2deg,shiftTurn,9
	currentPose.j9=currentPose.j9 - result()
	call lin2deg,shiftTurn,10
	currentPose.j10=currentPose.j10 + result()
S12	move P,tg=currentPose,spd=_mw344%,accu=0,tool=0 #선회 주행
	next
	*drive_align
	if di86 then *end_align #정렬 종료
	_mw76=415 #대차정렬 진행단계 - 최종 거리 정렬 시작
	currentPose=cpo("joint","cur")
	currentPose.j7=0
	currentPose.j8=0
S13	move P,tg=currentPose,spd=_mw346%,accu=0,tool=0 #주행 조향
	if di85 #전방 가까움
	_mw76=416 #대차정렬 진행단계 - 이격
	for idx=0 to 2000 step 1
	if di86 or di87 or di88
	_mw76=417 #대차정렬 진행단계 - 최종 이격 완료
	break
	endif
	currentPose=cpo("joint","cur")
	call lin2deg,_mw350,9
	currentPose.j9=currentPose.j9 - result()
	call lin2deg,_mw350,10
	currentPose.j10=currentPose.j10 - result()
S14	move P,tg=currentPose,spd=_mw340%,accu=2,tool=0 #이격 주행
	next
	elseif di87 #전방 멈
	_mw76=418 #대차정렬 진행단계 - 접근
	for idx=0 to 2000 step 1
	if di85 or di86
	_mw76=419 #대차정렬 진행단계 - 최종 접근 완료
	break
	endif
	currentPose=cpo("joint","cur")
	call lin2deg,_mw350,9
	currentPose.j9=currentPose.j9 + result()
	call lin2deg,_mw350,10
	currentPose.j10=currentPose.j10 + result()
S15	move P,tg=currentPose,spd=_mw340%,accu=2,tool=0 #일반 주행
	next
	endif
	*end_align
	currentPose=cpo("joint","cur")
	currentPose.j7=0
	currentPose.j8=0
S16	move P,tg=currentPose,spd=_mw346%,accu=0,tool=0 #조향 원위치
	_mw76=497 #대차정렬 진행단계 - 조향 원위치 완료
	_mw76=498 #대차정렬 진행단계 - 설정 쓰기
	gosub *setting
	_mw76=499 #대차정렬 진행단계 - 잡 완료
	_mw74=0 #잡 번호 초기화
end
	*setting #설정
	contpath 0
	_spd_rate=100 #재생속도 설정
	_tool[0]=Shift(_mw194/10,_mw196/10,_mw198/10,_mw200/10,_mw202/10,_mw204/10,"tool") #툴 데이터 설정
	var ucrdAssign=mkucs(1,Pose(_mw208/10,_mw210/10,_mw212/10,_mw214/10,_mw216/10,_mw218/10,0,0,0,0,"base")) #사용자 좌표계 데이터 설정
	seltool 0,robot #툴 선택
	selucrd 1 #사용자 좌표계 선택
	retsub


