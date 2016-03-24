# Atlas Joint Analysis

1. standing state machine

    _back_bky_ 躯干(torso)和骨盆(pelvis)之间的夹角
    
    _l_leg_hpy_ 骨盆和左大腿在sagittal平面内的夹角
    
    _r_leg_hpy_ 骨盆和右大腿在sagittal平面内的夹角
    
    _l_leg_kny_ 左大腿和左小腿(膝盖)之间在sagittal平面内的夹角
    
    _r_leg_kny_ 右大腿和右小腿(膝盖)之间在sagittal平面内的夹角
    
    _l_leg_aky_ 左小腿和左脚(脚踝)之间在sagittal平面内的夹角
    
    _r_leg_aky_ 右小腿和右脚(脚踝)之间在sagittal平面内的夹角

    _l_arm_shx_ 左大臂和躯干在coronal平面内的夹角
    
    _r_arm_shx_ 右大臂和躯干在coronal平面内的夹角
    
2. walking in place state machine
    
    _cd_ feedback gain of com (distance)
    
    _cv_ feedback gain of velocity 
    
3. update, stateMachine::computeControlForce, state::computeControlForce
    
    mTorque.head<6>() = zeros 不是因为要计算hip， 是因为这是第一个joint，连接world和pelvis
    
    可以将COM在sagitall和coronal平面拉回到stance leg(getStanceAnkelPosition)上
    > mDesiredJointPositionBalance = mDesiredJointPositions 
    >    
    >    \+ getSagitallCOMDistance() * mSagitalCd
    >
    >    \+ getSagitallCOMVelocity() * mSagitalCv
    >
    >    \+ getCoronalCOMDistance() * mCoronalCd
    >
    >    \+ getCoronalCOMVelocity() * mCoronalCv        
    
4. Walking State Machine 

    __State0__ : 抬右脚，nextState(State1), 0.3s
    
    __State1__ : 右脚落地, 左脚微抬， nextState(State2), rightfoot collide
    
    __State2__ : 抬左脚， nextState(State3), 0.3s
    
    __State3__ : 左脚落地， 右脚微抬， nextState(State0), left foot collide