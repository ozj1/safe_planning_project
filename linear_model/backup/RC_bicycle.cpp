#include "../flowstar/Continuous.h"

using namespace flowstar;
using namespace std;


int main(int argc, char *argv[])
//int main()
{
	unsigned int numVars = 10;

	int x_id = stateVars.declareVar("x");
	int y_id = stateVars.declareVar("y");
	int v_id = stateVars.declareVar("v");
        int theta_id = stateVars.declareVar("theta");
        int noise_id = stateVars.declareVar("noise");
        int x_noise_id = stateVars.declareVar("x_noise");
        int y_noise_id = stateVars.declareVar("y_noise");
        int v_noise_id = stateVars.declareVar("v_noise");
        int theta_noise_id = stateVars.declareVar("theta_noise");
        //int a11_id = stateVars.declareVar("a11");
        //int a12_id = stateVars.declareVar("a12");
	//int a13_id = stateVars.declareVar("a13");
	//int a14_id = stateVars.declareVar("a14");
	//int a21_id = stateVars.declareVar("a21");
	//int a22_id = stateVars.declareVar("a22");
	//int a23_id = stateVars.declareVar("a23");
	//int a24_id = stateVars.declareVar("a24");
	//int a31_id = stateVars.declareVar("a31");
	//int a32_id = stateVars.declareVar("a32");
	//int a33_id = stateVars.declareVar("a33");
	//int a34_id = stateVars.declareVar("a34");
	//int a41_id = stateVars.declareVar("a41");
	//int a42_id = stateVars.declareVar("a42");
	//int a43_id = stateVars.declareVar("a43");
	//int a44_id = stateVars.declareVar("a44"); 
	int t_id = stateVars.declareVar("t");

        std::string a11 = argv[1];
        std::string a12 = argv[2];
        std::string a13 = argv[3];
        std::string a14 = argv[4];
        std::string a21 = argv[5];
        std::string a22 = argv[6];
        std::string a23 = argv[7];
        std::string a24 = argv[8];
        std::string a31 = argv[9];
        std::string a32 = argv[10];
        std::string a33 = argv[11];
        std::string a34 = argv[12];
        std::string a41 = argv[13];
	std::string a42 = argv[14];
	std::string a43 = argv[15];
	std::string a44 = argv[16];
 
            
        double x_0_min = atof(argv[17]);
        double x_0_max = atof(argv[18]);
        double y_0_min = atof(argv[19]);
        double y_0_max = atof(argv[20]);
        double v_0_min = atof(argv[21]);
        double v_0_max = atof(argv[22]);
	double theta_0_min = atof(argv[23]);
	double theta_0_max = atof(argv[24]);
        double x_noise_min = atof(argv[25]);
        double x_noise_max = atof(argv[26]);
	double y_noise_min = atof(argv[27]);
	double y_noise_max = atof(argv[28]);
	double v_noise_min = atof(argv[29]);
	double v_noise_max = atof(argv[30]);
	double theta_noise_min = atof(argv[31]);
	double theta_noise_max = atof(argv[32]);
        //double noise = atof(argv[21]);
         
	// define the dynamics
        //Expression_AST<Real> ode_expression_x("a11*x+a12*y+a13*v+a14*theta+x_noise+noise");
        //Expression_AST<Real> ode_expression_x("1");
        Expression_AST<Real> ode_expression_x(a11+"*x+"+a12+"*y+"+a13+"*v+"+a14+"*theta+x_noise+noise");//v * cos(theta)
	//Expression_AST<Real> ode_expression_y("a21*x+a22*y+a23*v+a24*theta+y_noise+noise");
        Expression_AST<Real> ode_expression_y(a21+"*x+"+a22+"*y+"+a23+"*v+"+a24+"*theta+y_noise+noise");
        //Expression_AST<Real> ode_expression_y("1");//v * sin(theta) 
        //Expression_AST<Real> ode_expression_v("a31*x+a32*y+a33*v+a34*theta+v_noise+noise");
        //Expression_AST<Real> ode_expression_v("0");
        Expression_AST<Real> ode_expression_v(a31+"*x+"+a32+"*y+"+a33+"*v+"+a34+"*theta+v_noise+noise");
        //Expression_AST<Real> ode_expression_theta("a41*x+a42*y+a43*v+a44*theta+theta_noise+noise");
        //Expression_AST<Real> ode_expression_theta("0");
        Expression_AST<Real> ode_expression_theta(a41+"*x+"+a42+"*y+"+a43+"*v+"+a44+"*theta+theta_noise+noise");
        Expression_AST<Real> ode_expression_x_noise("0");
	Expression_AST<Real> ode_expression_y_noise("0");
	Expression_AST<Real> ode_expression_v_noise("0");
	Expression_AST<Real> ode_expression_theta_noise("0");
        Expression_AST<Real> ode_expression_noise("0");
	Expression_AST<Real> ode_expression_t("1");



	vector<Expression_AST<Real> > ode_rhs(numVars);
	ode_rhs[x_id] = ode_expression_x;
	ode_rhs[y_id] = ode_expression_y;
	ode_rhs[v_id] = ode_expression_v;
	ode_rhs[theta_id] = ode_expression_theta;
        ode_rhs[x_noise_id] = ode_expression_x_noise;
        ode_rhs[y_noise_id] = ode_expression_y_noise;
        ode_rhs[v_noise_id] = ode_expression_v_noise;
        ode_rhs[theta_noise_id] = ode_expression_theta_noise;
        ode_rhs[noise_id] = ode_expression_noise;
        //ode_rhs[a11_id] = ode_expression_a11;
        //ode_rhs[a12_id] = ode_expression_a12;
        //ode_rhs[a13_id] = ode_expression_a13;
        //ode_rhs[a14_id] = ode_expression_a14;
        //ode_rhs[a21_id] = ode_expression_a21;
        //ode_rhs[a22_id] = ode_expression_a22;
        //ode_rhs[a23_id] = ode_expression_a23;
        //ode_rhs[a24_id] = ode_expression_a24;
        //ode_rhs[a31_id] = ode_expression_a31;
        //ode_rhs[a32_id] = ode_expression_a32;
        //ode_rhs[a33_id] = ode_expression_a33;
        //ode_rhs[a34_id] = ode_expression_a34;
        //ode_rhs[a41_id] = ode_expression_a41;
        //ode_rhs[a42_id] = ode_expression_a42;
	//ode_rhs[a43_id] = ode_expression_a43;
	//ode_rhs[a44_id] = ode_expression_a44;
	ode_rhs[t_id] = ode_expression_t;



	Deterministic_Continuous_Dynamics dynamics(ode_rhs);



	// set the reachability parameters
	Computational_Setting setting;

	// set the stepsize and the order
	setting.setFixedStepsize(0.1, 2);
	//setting.setFixedStepsize(0.04, 5, 8);
	//setting.setAdaptiveStepsize(0.05, 2, 5);

	// set the time horizon
	setting.setTime(0.5);

	// set the cutoff threshold
	setting.setCutoffThreshold(1e-8);

	// set the queue size for the symbolic remainder, it is 0 if symbolic remainder is not used
	setting.setQueueSize(0);

	// print out the computation steps
	setting.printOn();

	// set up the remainder estimation
	Interval I(-0.1, 0.1);
	vector<Interval> remainder_estimation(numVars, I);
	setting.setRemainderEstimation(remainder_estimation);

	// call this function when all of the parameters are defined
	setting.prepare();


	// define the initial set which is a box
	//Interval init_x(0, 0.1), init_y(3, 3.1), init_v(15, 15.1),
	//		init_theta(0, 0.1), init_t;
	Interval init_x(x_0_min, x_0_max), init_y(y_0_min, y_0_max), init_v(v_0_min, v_0_max), init_theta(theta_0_min, theta_0_max), init_x_noise(x_noise_min, x_noise_max), init_y_noise(y_noise_min, y_noise_max), init_v_noise(v_noise_min, v_noise_max), init_theta_noise(theta_noise_min, theta_noise_max), init_noise(-0.01, 0.01), init_t;

	vector<Interval> initial_box(numVars);
	initial_box[x_id] = init_x;
	initial_box[y_id] = init_y;
	initial_box[v_id] = init_v;
	initial_box[theta_id] = init_theta;
        initial_box[x_noise_id] = init_x_noise;
        initial_box[y_noise_id] = init_y_noise;
        initial_box[v_noise_id] = init_v_noise;
        initial_box[theta_noise_id] = init_theta_noise;
        initial_box[noise_id] = init_noise;
        //initial_box[a11_id] = init_a11;
	//initial_box[a12_id] = init_a12;
	//initial_box[a13_id] = init_a13;
	//initial_box[a14_id] = init_a14;
	//initial_box[a21_id] = init_a21;
	//initial_box[a22_id] = init_a22;
	//initial_box[a23_id] = init_a23;
	//initial_box[a24_id] = init_a24;
	//initial_box[a31_id] = init_a31;
	//initial_box[a32_id] = init_a32;
	//initial_box[a33_id] = init_a33;
	//initial_box[a34_id] = init_a34;
	//initial_box[a41_id] = init_a41;
	//initial_box[a42_id] = init_a42;
	//initial_box[a43_id] = init_a43;
	//initial_box[a44_id] = init_a44;
	initial_box[t_id] = init_t;

	Flowpipe initialSet(initial_box);


	// empty unsafe set
	vector<Constraint> unsafeSet;
        //Constraint cons1("-0.79+2*x+2*y-x*x-y*y");
        //Constraint cons1("1.5556-x+y");
        //unsafeSet.push_back(cons1);

	/*
	 * The structure of the class Result_of_Reachability is defined as below:
	 * nonlinear_flowpipes: the list of computed flowpipes
	 * tmv_flowpipes: translation of the flowpipes, they will be used for further analysis
	 * fp_end_of_time: the flowpipe at the time T
	 */
	Result_of_Reachability result;

	// run the reachability computation
	//clock_t begin, end;
	//begin = clock();

	dynamics.reach(result, setting, initialSet, unsafeSet);
//	dynamics.reach(result, setting, result.fp_end_of_time, unsafeSet);
//	dynamics.reach(result, setting, result.fp_end_of_time, unsafeSet);

	//end = clock();
	//printf("time cost: %lf\n", (double)(end - begin) / CLOCKS_PER_SEC);

	// flowpipes should be translated to single Taylor model vectors before plotting
	result.transformToTaylorModels(setting);
                

	Plot_Setting plot_setting;
	plot_setting.printOff();
	plot_setting.setOutputDims(x_id, y_id);
        //int indicator = 1;
	//int indicator = plot_setting.plot_2D_interval_MATLAB("RC_bicycle", result);//Qin
        plot_setting.plot_2D_interval_MATLAB("RC_bicycle", result);
        //printf("indicator", (double)(indicator));
        //if(indicator == 0)
        //{
        //    cout<<x_0_min<<endl;
        //    cout<<y_0_min<<endl; 
        //}
        //cout<<indicator<<endl;
//	plot_setting.plot_2D_grids_GNUPLOT("test", 10, result);

	return 0;
}
