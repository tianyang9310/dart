# Cart Pole Balancing

```
	x0  = Eigen::Vector4d::Zero();
	x0(1)  = 1.2*M_PI;
	x0(3)  = -0.05*M_PI;


	Eigen::Vector4d	xd	= Eigen::Vector4d::Zero();
	xd(1)				= M_PI;

	Eigen::Matrix4d Q	= Eigen::Matrix4d::Zero();
	Q(0,0)				= 0.01;
	Q(1,1)				= 5;
	Q(2,2)				= 1;
	Q(3,3)				= 1;

	Eigen::Matrix<double,1,1> R;
	R(0,0)				= 1;

	Eigen::Matrix4d Qf = Eigen::Matrix4d::Identity();
	Qf(0,0)				= 1;
	Qf(1,1)				= 500;
	Qf(2,2)				= 100;
	Qf(3,3)				= 100;

    delta_t = 100;
	Q			= Q*delta_t;
	//R			= R*delta_t;
```

# Cart Pole Swing Up

```
	x0  = Eigen::Vector4d::Zero();
	x0(1)  = 0.0*M_PI;
	x0(3)  = -0.05*M_PI;


	Eigen::Vector4d	xd	= Eigen::Vector4d::Zero();
	xd(1)				= M_PI;

	Eigen::Matrix4d Q	= Eigen::Matrix4d::Zero();
	Q(0,0)				= 0.01;
	Q(1,1)				= 5;
	Q(2,2)				= 1;
	Q(3,3)				= 1;

	Eigen::Matrix<double,1,1> R;
	R(0,0)				= 1;

	Eigen::Matrix4d Qf = Eigen::Matrix4d::Identity();
	Qf(0,0)				= 1;
	Qf(1,1)				= 500;
	Qf(2,2)				= 100;
	Qf(3,3)				= 100;

    // delta_t = 100;
	Q			= Q*delta_t;
	R			= R*delta_t;

```
