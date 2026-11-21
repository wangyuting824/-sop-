import importlib.util,unittest
import numpy as np

HAS_CV=importlib.util.find_spec('cv2') is not None


@unittest.skipUnless(HAS_CV,'Install the vision extra to test OpenCV calculations')
class VisionTests(unittest.TestCase):
    def test_synthetic_intrinsics_holdout(self):
        import cv2
        from calibration_platform.compute.camera_intrinsics import solve
        K=np.array([[850.,0,640],[0,855,360],[0,0,1]])
        obj=np.array([[x*.035,y*.035,0] for y in range(6) for x in range(8)],dtype=np.float32)
        rows=[]
        for i in range(12):
            rv=np.array([.05+i*.025,(-1)**i*.15,.03*i]);t=np.array([-.12+.01*i,-.08,.6+.025*i])
            img,_=cv2.projectPoints(obj,rv,t,K,np.zeros(5))
            rows.append({'object_points_m':obj.tolist(),'image_points_px':img.reshape(-1,2).tolist(),
                         'split':'fit' if i<9 else 'holdout'})
        result=solve({'resolution':[1280,720],'views':rows})
        np.testing.assert_allclose(result['K'],K,rtol=2e-3,atol=.1)
        self.assertLess(max(result['holdout_reprojection_rmse_px']),.01)

    def test_handeye_ground_truth(self):
        import cv2
        from calibration_platform.compute.handeye import solve
        rng=np.random.default_rng(21)
        def T(rv,t):
            a=np.eye(4);a[:3,:3]=cv2.Rodrigues(np.asarray(rv,dtype=float))[0];a[:3,3]=t;return a
        X=T([.15,-.2,.1],[.03,.02,.1]);B=T([0,.1,0],[.5,.1,.4]);rows=[]
        for i in range(15):
            G=T(rng.uniform(-.7,.7,3),rng.uniform(-.2,.2,3))
            C=np.linalg.inv(X)@np.linalg.inv(G)@B
            rows.append({'index':i,'split':'fit' if i<10 else 'holdout','use_for_calibration':True,
                         'T_base_tool':{'matrix':G.tolist()},'T_camera_board':{'matrix':C.tolist()}})
        result=solve({'mode':'eye_in_hand','samples':rows})
        np.testing.assert_allclose(result['T_tool_camera'],X,atol=1e-6)

    def test_handeye_degenerate_motion_rejected(self):
        from calibration_platform.compute.handeye import solve
        rows=[{'index':i,'split':'fit' if i<10 else 'holdout','use_for_calibration':True,
               'T_base_tool':{'matrix':np.eye(4).tolist()},'T_camera_board':{'matrix':np.eye(4).tolist()}} for i in range(15)]
        with self.assertRaises(ValueError):solve({'mode':'eye_in_hand','samples':rows})


if __name__=='__main__':unittest.main()
