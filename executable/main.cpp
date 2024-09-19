#include "predictor.hpp"
#include <chrono>
#include <blaze/Blaze.h>
#include <blaze/Math.h>
#include <blaze/math/DenseMatrix.h>
#include <boost/tokenizer.hpp>
#include <thread>
#include <filesystem>

#include "TendonDriven.hpp"

static constexpr std::size_t dim_inputs = 1UL;
static constexpr std::size_t dim_lift = 2UL;
static constexpr std::size_t dim_states = 4UL;
static constexpr std::size_t dim_measurement = 2UL;

template <typename MatrixType>
MatrixType readFromCSV(MatrixType &Mat, const std::filesystem::path &filePath, bool skipHeader);

const static int NUM_SAMPLES = 10000UL;
const static double SAMPLE_TIME = 2e-3;
const static double SIM_TIME = NUM_SAMPLES * SAMPLE_TIME;

class CatheterModel
{
public:
    CatheterModel()
    {
        double E = 15.46795366474113845825195E9; // 22.00E9 Young's modulus [GPa] 15.46795366474113845825195E9
        double nu = 0.385;                       // Poisson's ratio --> for Nitinol: [0.30, 0.50]
        double G = E / (2.00 * (1.00 + nu));     // Shear modulus [GPa]
        double radius = 4.001769660928E-4;       // radius of center beam [m] -- 4.6769660928E-4;
        double mass = 0.516E-3;                  // total mass of the sheath [kg]
        double Length = 0.05436;                 // total length of the sheath [m]
        double T = SIM_TIME;                     // simulation time [sec]
        double dt = SAMPLE_TIME;                 // time infinitesimal [sec]
        double basePlateToMotor = 1.19;          // distance between base plate and motor [m]
        double tendonOffset = 1.112394E-3;       // offset distance between the tendons and backbone [m]
        double tendonCompliance = 4.65937E-4;    // tendon compliance to tensile forces -- MADE EQUAL FOR ALL TENDONS 2.52E-4;
        double alpha = -0.50;                    // backward differentiation parameter BDF-alpha ==> alpha in [-0.5, 0.00]
        double dampingBendTwist = 1.50E-6;       // viscous damping for bending & twist 0.85E-6;
        double dampingShearExt = 7.50E-5;        // viscous damping for shear & extension
        double airDragCoeff = 0.00;              // viscous air drag coefficient -- 1.81E-6

        // //  # # # # # # # # ---- Forced system - Properties of the Nitinol Backbone ---- # # # # # # # #
        // double E = 15.46795366474113845825195E9; // 22.00E9 Young's modulus [GPa] 15.46795366474113845825195E9
        // double nu = 0.385;						 // Poisson's ratio --> for Nitinol: [0.30, 0.50]
        // double G = E / (2.00 * (1.00 + nu));	 // Shear modulus [GPa]
        // double radius = 4.001769660928E-4;		 // radius of center beam [m] -- 4.6769660928E-4;
        // double mass = 0.516E-3;					 // total mass of the sheath [kg]
        // double Length = 0.055;					 // total length of the sheath [m]
        // double T = SIM_TIME;
        // double dt = SMAPLE_TIME;		   // time infinitesimal [sec]
        // double basePlateToMotor = 1.19;	   // distance between base plate and motor [m]
        // double tendonOffset = 1.12E-3;	   // offset distance between the tendons and backbone [m]
        // double tendonCompliance = 2.52E-4; // tendon compliance to tensile forces -- MADE EQUAL FOR ALL TENDONS 2.52E-4;
        // double alpha = 0.00;			   // backward differentiation parameter BDF-alpha ==> alpha in [-0.5, 0.00]
        // double dampingBendTwist = 4.20E-5;		 // viscous damping for bending & twist 0.85E-6;
        // double dampingShearExt = 7.50E-6; // viscous damping for shear & extension
        // double airDragCoeff = 2.81E-5;	  // viscous air drag coefficient

        const double distalMass = 00.00E-3;                 // 30 grams weight at the end of the catheter
        const double probeMass = 340.00E-6;                 // 339.00E-6;					// mass of the ultrasound probe at the distal end of the catheter 0.339 gram
        const double g = -9.81;                             // acceleration og gravity [m/s/s]
        const double weight = (probeMass + distalMass) * g; // weight [Newtons]
        blaze::StaticVector<double, 3UL> distalForce = {weight, 0.00, 0.00};

        /****************************************************************************************************
         * 		INSTANTIATING A 4-TENDON CATHETER OBJECT WITH 200 DISCRETIZED POINTS ALONG ITS BACKBONE		*
         *****************************************************************************************************/

        // asserts if alpha parameter for implicit time differentiation is valid
        auto isWithinRange = [](double alpha) -> bool
        {
            if (alpha >= -0.50 && alpha <= 0.00)
            {
                return true;
            }
            else
            {
                std::cerr << "Execution Interrupted ==> BDF-alpha parameter does not lie within allowed interval [-0.50, 0.00]!" << std::endl;
                exit(1);
            }
        };

        isWithinRange(alpha);

        // number of time steps to consider
        const size_t timeSteps = static_cast<size_t>(T / dt);

        // instantiating a tendon-driven robot (4 tendons, 200 discretized arc-length points)
        // TendonDriven<backboneDiscretizedPoints, numberOfTendons> robot(E, G, radius, mass, Length, alpha, dt, tendonOffset, tendonCompliance, basePlateToMotor, dampingBendTwist, dampingShearExt, airDragCoeff);
        robot_ = std::make_unique<TendonDriven<backboneDiscretizedPoints_, numberOfTendons_>>(E, G, radius, mass, Length, alpha, dt, tendonOffset, tendonCompliance, basePlateToMotor, dampingBendTwist, dampingShearExt, airDragCoeff);
        robot_->setExternalDistalForce(distalForce);

        // setting the actuation input at time t = 0.00 second
        q = blaze::StaticVector<double, numberOfTendons_>(0.0);
        robot_->setTendonActuation(q);
        bool convergence;
        // solve the static problem -> initial condition for the dynamics problem
        convergence = robot_->solveBVP<mathOp::cosseratEquations::STATIC_SOLUTION, mathOp::rootFindingMethod::MODIFIED_NEWTON_RAPHSON, mathOp::integrationMethod::RK4>(initGuess_);
        if (!convergence)
        {
            initGuess_ *= 0.75;
            convergence = robot_->solveBVP<mathOp::cosseratEquations::STATIC_SOLUTION, mathOp::rootFindingMethod::MODIFIED_NEWTON_RAPHSON, mathOp::integrationMethod::RK4>(initGuess_);
        }
        if (!convergence)
        {
            initGuess_ *= 0.25;
            convergence = robot_->solveBVP<mathOp::cosseratEquations::STATIC_SOLUTION, mathOp::rootFindingMethod::MODIFIED_NEWTON_RAPHSON, mathOp::integrationMethod::RK4>(initGuess_);
            if (!convergence)
                std::cout << "Static model still hasn't converged yet!" << std::endl;
        }

        // set the parameters for the implicit differentiation for time discretization of PDEs
        robot_->setTimeDiscretization();

        // std::cout << "Starting point: " << startingPoint << std::endl;

        // robot.setExternalDistalForce(distalForce);
    };

    ~CatheterModel()
    {
        robot_->resetStaticSimulation();
        robot_->resetDynamicSimulation();
    }

    // step one time step
    void simulationStep(blaze::StaticVector<double, 2UL> q)
    {
        // lines bellow are for wight release
        // if (counter == 0)
        // {
        //   const double probeMass = 340.00E-6;                 // 339.00E-6;					// mass of the ultrasound probe at the distal end of the catheter 0.339 gram
        //   const double g = -9.81;                             // acceleration og gravity [m/s/s]
        //   // release weight from the catheter's distal end
        //   blaze::StaticVector<double, 3UL> distalForce = {probeMass * g, 0.00, 0.00};
        //   robot->setExternalDistalForce(distalForce);
        // }

        bool convergence;
        robot_->setTendonActuation(q);
        // solve the static problem
        convergence = robot_->solveBVP<mathOp::cosseratEquations::DYNAMIC_SOLUTION, mathOp::rootFindingMethod::MODIFIED_NEWTON_RAPHSON, mathOp::integrationMethod::RK4>(initGuess_);
        if (!convergence)
        {
            initGuess_ = 0.00;
            convergence = robot_->solveBVP<mathOp::cosseratEquations::DYNAMIC_SOLUTION, mathOp::rootFindingMethod::MODIFIED_NEWTON_RAPHSON, mathOp::integrationMethod::RK4>(initGuess_);
        }
        robot_->stepTime();
    }

    // step one time step
    void getOutput(blaze::StaticVector<double, 3UL> &pos, blaze::StaticVector<double, 3UL> &vel)
    {
        // record tip position
        pos = std::get<0>(robot_->getTrainingData());
        vel = std::get<1>(robot_->getTrainingData());
        // blaze::column(tendon_tau, k) = std::get<2>(robot_->getTrainingData());
        // blaze::column(tendon_disp, k) = std::get<3>(robot_->getTrainingData());
    }

private:
    // Member variables
    static const size_t backboneDiscretizedPoints_ = 32UL;                                                                    // defining the spatial resolution
    static const size_t numberOfTendons_ = 2UL;                                                                               // number of tendons present in the sheath
    std::unique_ptr<TendonDriven<backboneDiscretizedPoints_, numberOfTendons_>> robot_;                                       // Robot object
    blaze::StaticVector<double, 6UL + numberOfTendons_> initGuess_;                                                           // initial guess for the BVP problem
    blaze::StaticVector<double, 3UL> distalPositions_ = blaze::StaticVector<double, 3UL>(0.0);                                // distal position
    blaze::StaticVector<double, 19UL + numberOfTendons_> states_ = blaze::StaticVector<double, 19UL + numberOfTendons_>(0.0); // full states_
    blaze::StaticVector<double, numberOfTendons_> q = blaze::StaticVector<double, numberOfTendons_>(0.0);                     // tendons pull
    size_t count_;
    double t0_ = 0.0;
    double dt_, dt_publisher_;
};

int main()
{
    

    static const size_t numberOfTendons = 2UL;
    // const double sampleTime = 2e-3;
    blaze::StaticVector<double, numberOfTendons> q = blaze::StaticVector<double, numberOfTendons>(0.0);

    blaze::StaticVector<double, 3UL> pos, vel;

    // Initialize Koopman predictor and Cosserat model objects
    auto robot = CatheterModel();
    Predictor simulator(SAMPLE_TIME, 1UL, 500.0f);

    // Initialize output files
    std::filesystem::path workspace_directory = WORKSPACE_DIR;
    std::filesystem::path filePath_output = workspace_directory / "output_files" / "sim_output.csv";
    std::filesystem::path filePath_matrices = workspace_directory / "output_files" / "matrices.csv";
    std::ofstream csv_file_output(filePath_output);
    if (!csv_file_output.is_open())
    {
        std::cerr << "Failed to open the file for writing" << std::endl;
        return -1;
    }
    csv_file_output << "t,u,pred_p_x,pred_p_y,pred_p_z,pred_v_x,pred_v_y,pred_v_z,p_x,p_y,p_z,v_x,v_y,v_z\n";

    std::ofstream csv_file_matrices(filePath_matrices);
    if (!csv_file_matrices.is_open())
    {
        std::cerr << "Failed to open the file for writing" << std::endl;
        return -1;
    }
    csv_file_matrices << "t,A11,A12,A21,A22,B11,B21\n";

    // Initialize variabels
    blaze::StaticVector<double, dim_states> X, X_pred, X0;                                    // Robot states [mm] and [mm/s]
    blaze::StaticVector<double, dim_inputs> U = blaze::StaticVector<double, dim_inputs>(0.0); // Control inputs [mm]
    blaze::StaticVector<double, dim_lift> Y;                                                  // Lifted states
    blaze::StaticMatrix<float, dim_lift, dim_lift> A;
    blaze::StaticMatrix<float, dim_lift, dim_inputs> B;

    X0 = {-1.780, 0.000, 54.471, 0.000};
    X = X0;

    // multi sine trajectory parameters
    double frequency = 0.5;
    double amplitude = 5.0;

    simulator.encode(X);

    // Iterate
    std::size_t count = 0;

    while (count < 15000)
    {
        double t = count * SAMPLE_TIME;
        if (t > 1)
        {
            U[0] = amplitude * (0.0 + 0.5 * (std::sin(2 * M_PI * frequency * 1 / 7 * (t - 1))));
            U[0] += amplitude * (0.0 + 0.3 * (std::sin(2 * M_PI * frequency * 1 / 3 * (t - 1))));
            U[0] += amplitude * (0.0 + 0.2 * (std::sin(2 * M_PI * frequency * 1 / 1 * (t - 1))));
        }
        else
        {
            U[0] = 0.0;
        }

        if (count > 100)
        {
            // ### simulate using Cosserat's rod catheter model
            q = {U[0] * 1e-3, -1 * U[0] * 1e-3};
            robot.simulationStep(q);
            robot.getOutput(pos, vel);
            X = {pos[0] * 1e3, vel[0] * 1e3, pos[2] * 1e3, vel[2] * 1e3};

            // ### simulate using DeepKoopman catheter model
            simulator.update_parameters();
            A = simulator.get_A();
            B = simulator.get_B_phi();
            simulator.koopman_step(U);
            Y = simulator.get_lifted_states();
            X_pred = simulator.decode(Y);

            // Write the tensor `x` to the CSV file
            csv_file_output
                // << elapsed_2.count() << ","
                << count * SAMPLE_TIME << ","
                << U[0] << ","
                << X_pred[0] << ","
                << 0.0 << ","
                << X_pred[2] << ","
                << X_pred[1] << ","
                << 0.0 << ","
                << X_pred[3] << ","
                << X[0] << ","
                << 0.0 << ","
                << X[2] << ","
                << X[1] << ","
                << 0.0 << ","
                << X[3] << "\n";

            // Write the matrices to the CSV file
            csv_file_matrices
                // << elapsed_2.count() << ","
                << count * SAMPLE_TIME << ","
                << A(0,0) << ","
                << A(0,1) << ","
                << A(1,0) << ","
                << A(1,1) << ","
                << B(0,0) << ","
                << B(1,0) << "\n";


            std::cout << std::fixed << std::setprecision(2);
            std::cout
                << "x_1: " << X_pred[0] << "  "
                << "x_2: " << X_pred[1] << "  "
                << "x_3: " << X_pred[2] << "  "
                << "x_4: " << X_pred[3] << "  |  "
                << "x_kpm_1: " << X[0] << "  "
                << "x_kpm_2: " << X[1] << "  "
                << "x_kpm_3: " << X[2] << "  "
                << "x_kpm_4: " << X[3] << "  |  "
                << std::endl;
        }
        count++;
    }

    // Close the CSV file
    csv_file_output.close();

    return 0;
}

// function that reads data from CSV files
template <typename MatrixType>
MatrixType readFromCSV(MatrixType &Mat, const std::filesystem::path &filePath, bool skipHeader)
{
    std::ifstream CSV_file;
    CSV_file.open(filePath, std::ifstream::in);
    if (!CSV_file.is_open())
    {
        throw std::runtime_error("Error opening the CSV file: " + filePath.string());
    }

    typedef boost::tokenizer<boost::escaped_list_separator<char>> Tokenizer;
    std::string line;

    // Conditionally skip the first line (header)
    if (skipHeader)
    {
        if (!std::getline(CSV_file, line))
        {
            std::cerr << "Error reading the header line\n";
            return Mat = -1.00;
        }
    }

    size_t row = 0UL, col = 0UL;
    double value;

    while (std::getline(CSV_file, line))
    {
        Tokenizer tokenizer(line);
        col = 0UL;

        for (Tokenizer::iterator it = tokenizer.begin(); it != tokenizer.end(); ++it)
        {
            value = std::stod(*it);
            Mat(row, col) = value;
            ++col;
        }
        ++row;
    }

    CSV_file.close();
    Mat.resize(row, col, true);

    return Mat;
}
