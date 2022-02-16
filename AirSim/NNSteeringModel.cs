//using VePack.Utilities;
//using VePack.Plugin.Controllers;

//namespace AirSim
//{

//    public class NNSteeringModel : SteeringModel
//    {

//        // ------ fields ------ //

//        private readonly Onnx _onnx;


//        // ------ constructors ------ //

//        public NNSteeringModel(string model, double lf, double lr, double timeConstant, double dt = 0.1) 
//            : base(lf, lr, timeConstant, dt)
//        {
//            _onnx = new(model, Provider.CPU, OptimizationLevel.Off);
//        }


//        // ------ public methods ------ //

//        public void SetParams(double lateralError, Angle headingError, Angle steering, double speed, double cuvature = 0)
//        {
//            SetParams(speed, cuvature);
//            var input = new float[] { (float)lateralError, (float)headingError.Radian, (float)steering.Radian, (float)speed };
//            var r = _onnx.Run(input);
//            A[0, 1] = r["a01"][0];
//            A[0, 2] = r["a02"][0];
//            A[1, 2] = r["a12"][0];
//       }

//    }
//}
