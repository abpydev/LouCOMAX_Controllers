import time
import scipy

from corapp.controllers.amptek_controller import AmptekDevice
from utils.hdf5_utils import Hdf5Handler
from utils.hdf5_utils import Hdf5Handler

class Nanomax300Controller():

    def __init__(self, amptek_controller) -> None:
        self.amptek_controller = amptek_controller
        self.evaluation_dwell_time = 50
        self.evaluation_roi = [0, 511]
        self.evaluation_data = None

    def start_connection(self):
        raise NotImplementedError

    def is_all_axes_homed(self):
        raise NotImplementedError

    def measure_point_count(self, xyz_coord, *args) -> float :

        amptek_controller: AmptekDevice             = args[1]
        dwell_time_ms: int                          = args[2]
        roi_chan: list[int, int]                    = args[3]

        if not self.is_all_axes_homed():
            self.home_all_axes()

        self.move_to(xyz_coord[0], xyz_coord[1], xyz_coord[2])
        amptek_controller.get_spectrum(get_status=False, clear_spectrum=True)
        time.sleep(dwell_time_ms / 10**3)
        spectrum = amptek_controller.get_spectrum(get_status=False, clear_spectrum=True)
        
        count_result = sum(spectrum[roi_chan[0]:roi_chan[1]])

        return count_result

    def measure_point_count_simulate(xyz_coord:tuple[float, float, float], *args) -> float :
        # return self.measure_point_count(self.amptek_controller, x_coord, y_coord, z_coord, self.evaluation_dwell_time, self.evaluation_roi)
        simulation_data = Hdf5Handler.get_dataset_data_hdf5(
            r"C:\Users\antoi\OneDrive\Documents\3 - Code\CNRS\ABK_Workspace\CXRF_REPO\depthpaint-c-xrf-interface\tests\data\11-14-2024\Scan_cube_modif_meca4\test_confocal_align.hdf5",
            group_name="Test confocal alignment",
            dataset_name="data")
        
        bounds = ((0,9),(0,19),(0,5))
        
        xyz0 = simulation_data[int(xyz_coord[0])][int(xyz_coord[1])][int(xyz_coord[2])]
        x_pond = float(xyz_coord[0] - round(xyz_coord[0]))
        if xyz_coord[0] == bounds[0][0] or xyz_coord[0] == bounds[0][1]:
            x1yz = xyz0
        else :
            x1yz = (1-x_pond) * xyz0 + x_pond * simulation_data[int(xyz_coord[0])+1][int(xyz_coord[1])][int(xyz_coord[2])]
        
        y_pond = xyz_coord[1] - round(xyz_coord[1])
        if xyz_coord[1] == bounds[1][0] or xyz_coord[1] == bounds[1][1]:
            xy1z = xyz0
        else :
            xy1z = (1-y_pond) * xyz0 + y_pond * simulation_data[int(xyz_coord[0])][int(xyz_coord[1]+1)][int(xyz_coord[2])]
        
        z_pond = xyz_coord[2] - round(xyz_coord[2])
        if xyz_coord[2] == bounds[2][0] or xyz_coord[2] == bounds[2][1]:
            xyz1 = xyz0
        else :
            xyz1 = (1-z_pond) * xyz0 + z_pond * simulation_data[int(xyz_coord[0])][int(xyz_coord[1])][int(xyz_coord[2]+1)]
        
        # point_data = simulation_data[int(xyz_coord[0])][int(xyz_coord[1])][int(xyz_coord[2])]

        point_data = (x1yz + xy1z + xyz1) / 3
        
        if point_data == 0 :
            print(f"{xyz_coord} inf")
            return float("inf")
        else :
            print(f"{xyz_coord}", float(1/point_data))
            return float(1/point_data)

    def search_max(self, xyz0, method='Nelder-Mead', bounds=None):
        res = scipy.optimize.minimize(
            self.measure_point_count_simulate, 
            xyz0, 
            method=method, 
            bounds=bounds,
            options={'return_all':True, 'initial_simplex': [[5,11,2], [6,13,4], [8,13,2], [1,7,4]]})
        print(res)

def main():

    nanomax_controller = Nanomax300Controller(None)

    nanomax_controller.search_max((5, 11, 2), bounds=((0,9),(0,19),(0,5)), method='nelder-mead')
    # nanomax_controller.search_max((5, 11, 2), bounds=((0,10),(0,20),(0,6)))
    # res = Nanomax300Controller.measure_point_count_simulate((1, 1, 1))
    # print(res)
    # res = Nanomax300Controller.measure_point_count_simulate((2, 2, 2))
    # print(res)
    # res = Nanomax300Controller.measure_point_count_simulate((5, 5, 5))
    # print(res)
    # res = Nanomax300Controller.measure_point_count_simulate((1, 7, 4))
    # print(res)


if __name__ == "__main__" :
    main()