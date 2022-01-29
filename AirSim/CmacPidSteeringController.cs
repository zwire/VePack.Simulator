using System;
using System.Collections.Generic;
using Husty.Cmac;
using VePack.Plugin.Controllers;
using static System.Math;

namespace AirSim
{
    public class CmacPidSteeringController : Pid
    {

        // ------ fields ------ //

        private bool _freeze, _flush;
        private double _preU, _uDiff;
        private readonly CmacNetwork _pNet, _iNet, _dNet;


        // ------ properties ------ //

        public bool IsFreezed => _freeze;


        // ------ constructors ------ //

        public CmacPidSteeringController(
            CmacParam kp,
            CmacParam ki,
            CmacParam kd,
            IEnumerable<CmacLabelInfo> labels,
            int tableCount,
            bool freeze = false
        ) : base(PidType.Speed, kp.InitialValue, ki.InitialValue, kd.InitialValue)
        {
            _freeze = freeze;
            _pNet = new(tableCount, labels, kp);
            _iNet = new(tableCount, labels, ki);
            _dNet = new(tableCount, labels, kd);
        }


        // ------ public methods ------ //

        public override double GetControlQuantity(double error)
        {
            var state = new[] { error, PreviousError };
            var dedu = _uDiff == 0 ? 0 : (error - PreviousError) / (Sign(_uDiff) + _uDiff);
            var absE = Abs(error);
            if (_flush)
            {
                dedu = 0;
                _flush = false;
            }
            if (!_freeze)
            {
                _pNet.Backward(state, absE, -(absE - Abs(PreviousError)) * dedu);
                _iNet.Backward(state, absE, -absE * dedu);
                _dNet.Backward(state, absE, -(absE - 2.0 * Abs(PreviousError) + Abs(PreviousError2)) * dedu);
            }
            Kp = _pNet.Forward(state);
            Ki = _iNet.Forward(state);
            Kd = _dNet.Forward(state);
            var u = base.GetControlQuantity(error);
            _uDiff = u - _preU;
            _preU = u;
            return u;
        }

        public void Flush() => _flush = true;

        public void Freeze() => _freeze = true;

        public void Unfreeze() => _freeze = false;
    }
}
