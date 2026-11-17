import unittest
from dataclasses import replace
from pathlib import Path
import tempfile
import numpy as np

from two_tank.controllers import ControllerParameters, command, integral_derivative
from two_tank.experiments import run_experiments
from two_tank.model import (TankParameters, open_loop_equilibrium,
                            requested_link_flow, requested_outlet_flow,
                            target_equilibrium)
from two_tank.simulation import Scenario, simulate


class TwoTankTests(unittest.TestCase):
    def setUp(self): self.tank=TankParameters()

    def test_initial_flows_and_slopes(self):
        q12=requested_link_flow(2,.5,self.tank);qout=requested_outlet_flow(.5,self.tank)
        self.assertAlmostEqual(q12,.018);self.assertAlmostEqual(qout,.004)
        self.assertAlmostEqual((.04-q12)/2,.011)
        self.assertAlmostEqual((q12-qout)/1.5,.009333333333333)

    def test_one_euler_step(self):
        r=simulate(self.tank,ControllerParameters(mode="open_loop"),Scenario(duration=.1,dt=.1))
        self.assertAlmostEqual(r["h1"][1],2.0011)
        self.assertAlmostEqual(r["h2"][1],.500933333333333)

    def test_reference_equilibria(self):
        self.assertEqual(open_loop_equilibrium(.04,self.tank),(8.333333333333334,5.0))
        self.assertEqual(target_equilibrium(3,0,self.tank),(5.0,3,.024))
        self.assertEqual(target_equilibrium(3,.006,self.tank),(5.5,3,.03))

    def test_one_way_link(self):
        self.assertEqual(requested_link_flow(1,2,self.tank),0)
        self.assertEqual(requested_link_flow(2,2,self.tank),0)

    def test_controller_reads_height2(self):
        c=ControllerParameters(mode="p")
        error,raw,_,_=command(1,0,c)
        self.assertEqual(error,2);self.assertAlmostEqual(raw,.064)

    def test_p_equilibrium_and_long_run(self):
        expected=.078/.028
        h1=expected+(self.tank.outlet_coefficient*expected+.006)/self.tank.link_coefficient
        equilibrium=replace(self.tank,initial_height1=h1,initial_height2=expected)
        r=simulate(equilibrium,ControllerParameters(mode="p"),Scenario(duration=10,disturbance_at=0,demand=.006))
        np.testing.assert_allclose(r["h2"],expected,atol=1e-10)
        self.assertGreater(3-expected,0)

    def test_pi_disturbed_equilibrium(self):
        tank=replace(self.tank,initial_height1=5.5,initial_height2=3)
        r=simulate(tank,ControllerParameters(mode="pi"),Scenario(duration=10,disturbance_at=0,demand=.006,initial_integral=60))
        np.testing.assert_allclose(r["h1"],5.5,atol=1e-10);np.testing.assert_allclose(r["h2"],3,atol=1e-10)
        np.testing.assert_allclose(r["pump_actual"],.03,atol=1e-12)

    def test_mass_balance_every_step(self):
        r=simulate(self.tank,ControllerParameters(mode="pi"),Scenario(duration=100,disturbance_at=20,demand=.006))
        self.assertLess(np.max(np.abs(r["volume_balance_residual"])),1e-12)
        self.assertTrue(np.all(r["h1"]>=0));self.assertTrue(np.all(r["h2"]>=0))

    def test_dry_tank_reports_unmet_demand(self):
        tank=replace(self.tank,initial_height1=0,initial_height2=0)
        c=ControllerParameters(mode="open_loop",fixed_flow=0)
        r=simulate(tank,c,Scenario(duration=1,disturbance_at=0,demand=.006))
        self.assertTrue(np.all(r["demand_served"]==0));self.assertTrue(np.all(r["demand_unmet"]==.006))
        self.assertLess(np.max(np.abs(r["volume_balance_residual"])),1e-12)

    def test_anti_windup_blocks_and_unwinds(self):
        c=ControllerParameters(mode="pi")
        self.assertEqual(integral_derivative(1,.2,c),(0,False))
        self.assertEqual(integral_derivative(-1,.2,c),(-1,True))
        self.assertEqual(integral_derivative(-1,-.1,c),(0,False))
        self.assertEqual(integral_derivative(1,-.1,c),(1,True))

    def test_disturbance_switch_and_alignment(self):
        r=simulate(self.tank,ControllerParameters(mode="p"),Scenario(duration=1,dt=.1,disturbance_at=.5,demand=.006))
        np.testing.assert_allclose(r["demand_requested"][:5],0);np.testing.assert_allclose(r["demand_requested"][5:],.006)
        self.assertEqual(len(r["time"]),11);self.assertEqual(len(r["pump_actual"]),10)

    def test_validation(self):
        for action in (lambda:TankParameters(area1=0),lambda:ControllerParameters(mode="bad"),lambda:Scenario(dt=0),lambda:Scenario(duration=1,dt=.3)):
            with self.assertRaises(ValueError): action()

    def test_experiments_create_plots(self):
        with tempfile.TemporaryDirectory() as d:
            run_experiments(Path(d))
            names=("open_loop_two_tank.png","tracking_comparison.png","disturbance_rejection.png","anti_windup.png")
            for name in names:self.assertGreater((Path(d)/name).stat().st_size,1000)


if __name__=="__main__":unittest.main()
