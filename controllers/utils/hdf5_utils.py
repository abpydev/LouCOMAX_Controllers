import h5py
import numpy as np
import threading
from datetime import datetime
import matplotlib.pyplot as plt

# Static Class
class Hdf5Handler():

    _FILE_LOCK_1 = threading.Lock()

    @staticmethod
    def save_hdf5(filename, points_map, spectrums_list, group_name="XRF_Analysis", dataset_name="dataset"):
        try :
            assert(len(points_map) == len(spectrums_list))
        except AssertionError:
            raise(IndentationError(f"points_map length : {len(points_map)} and spectrum_list length : {len(spectrums_list)} do not match."))
        
        with h5py.File(filename, 'w') as h5file:
            
            data = np.zeros((3, 3, 3, len(spectrums_list[0])))

            for index, point in enumerate(points_map) :
                x = point[0]
                y = point[1]
                z = point[2]
                data[x,y,z] = spectrums_list[index]
            
            subgroup = h5file.require_group(group_name)
            subgroup.attrs['Analysis date'] = datetime.now().strftime('%d/%m/%Y')
            subgroup.attrs['Analysis time'] = datetime.now().strftime("%H:%M:%S")
            dset = subgroup.create_dataset(dataset_name, data=data)

    @staticmethod
    def create_empty_hdf5(filename: str, data_shape=None, dtype=np.float64, group_name=None, dataset_name=None, file_lock=_FILE_LOCK_1):
        with file_lock :
            with h5py.File(filename, 'w') as h5file:
                if group_name is None or data_shape is None or dataset_name is None :
                    pass
                else:
                    subgroup = h5file.require_group(group_name)
                    dset = subgroup.create_dataset(dataset_name, shape=data_shape, dtype=dtype)

    @staticmethod
    def feed_existing_hdf5(filename, data, group_name="XRF_analysis", dataset_name="dataset", file_lock=_FILE_LOCK_1):
        """Feeds a new line to preexisting HDF5 File
        
        The file must be created before calling this method
        """
        with file_lock :
            with h5py.File(filename, 'a') as h5file:
                dset = h5file[f'{group_name}/{dataset_name}']
                dset[:] = data

    @staticmethod
    def get_dataset_data_hdf5(filename, group_name:str="XRF_analysis", dataset_name:str="dataset", file_lock=_FILE_LOCK_1):
        
        with file_lock :
            with h5py.File(filename, 'r') as h5file:
                group = h5file.require_group(f'{group_name}')
                dset_data = group[f'{dataset_name}']
                np_dset_data = np.array(dset_data)

            return np_dset_data

    @staticmethod
    def feed_spectrum(filename, spectrum:list[int], x_position:int, y_position:int, group_name="XRF_analysis", dataset_name="dataset", file_lock=_FILE_LOCK_1) -> None :
        try :
            with file_lock : 
                with h5py.File(filename, 'a') as h5file:
                    group = h5file.require_group(f'{group_name}')
                    dset = group[f'{dataset_name}']
                    dset[y_position, x_position] = spectrum
        except IndexError as idxerr :
            print(f'INDEXERROR : {x_position} or {y_position} out of range \n {idxerr}')
            raise IndexError(idxerr)

    @staticmethod
    def save_data_to_hdf5(save_filepath : str,
                                output_data: np.ndarray,
                                 calibration: np.ndarray,
                                  project_name="Project",
                                   object_name="Object",
                                    analysis_name="XRF_Analysis",
                                     dataset_name="C-XRF Profile",
                                      mono_xrf_dataset_name="XRF point",
                                       mono_xrf_data= None,
                                        calibration_mono_xrf= None,
                                         picture=None,
                                          metadata:dict={}):
        """Save the data from MAXRF mapping to a given HDF5 file"""

        with h5py.File(save_filepath, 'a') as final_hdf5:

            project_group = final_hdf5.require_group(project_name)
            object_group = project_group.require_group(object_name)
            profile_group = object_group.require_group(analysis_name)
            mono_xrf_group_name = str(profile_group.name).replace("C-XRF", "XRF")
            mono_xrf_group = object_group.require_group(mono_xrf_group_name)
            try:
                # Create the dataset if it does not exist
                dataset = profile_group.require_dataset(dataset_name, shape=output_data.shape, dtype=output_data.dtype)
                
                # Save the picture if needed
                if picture is not None :
                    picture_dset = profile_group.create_dataset(f'Snapshot', data=picture, compression='gzip')
                    picture_dset.attrs.create('CLASS','IMAGE',dtype='S6')
                    picture_dset.attrs.create('IMAGE_SUBCLASS','IMAGE_TRUECOLOR',dtype='S16')

            except TypeError :
                # Dataset already exists, we then append the data
                dataset = profile_group.get(dataset_name)

            dataset[:] = output_data

            calib = profile_group.require_dataset("calibration", shape=calibration.shape, dtype=np.float64)
            calib[:] = calibration

            # If Mono XRF data is provided, create a dataset for it
            # and copy the data into it
            if mono_xrf_data is not None:
                try:
                    # Create the dataset if it does not exist
                    mono_xrf_dataset = mono_xrf_group.require_dataset(mono_xrf_dataset_name, shape=(512,), dtype=np.float64)
                except TypeError as e:
                    # Dataset already exists, we then append the data
                    print(f"TypeError : {e}")
                    mono_xrf_dataset = mono_xrf_group.get(mono_xrf_dataset_name)

                mono_xrf_dataset[:] = mono_xrf_data

                calib_mono = mono_xrf_group.require_dataset("calibration", shape=(3,), dtype=np.float64)
                calib_mono[:] = calibration_mono_xrf

            for attribute, value in metadata.items() :
                if value is None:
                    value = ''
                if "project" in attribute.lower() or "user" in attribute.lower():
                    project_group.attrs[attribute] = value
                elif "object" in attribute.lower() or "sample" in attribute.lower():
                    object_group.attrs[attribute] = value
                else:
                    profile_group.attrs[attribute] = value
                    mono_xrf_group.attrs[attribute] = value


    @staticmethod
    def visualize_3d_mapping(hdf5_filepath, dataset_shape=None, group_name="Test confocal alignment", dataset_name="dataset"):
        with h5py.File(hdf5_filepath, 'r') as hdf5_file:
            data_group = hdf5_file.get(group_name)
            # data_cube = data_group.require_dataset(dataset_name, shape=dataset_shape, dtype=np.int32, exact=False)
            data_cube = data_group.get(dataset_name)
            if data_cube is None :
                raise ValueError(f"Could not find dataset {dataset_name} in group {group_name} inside hdf5file : {hdf5_filepath}")
            

            x = list(np.arange(0, data_cube.shape[0])) * data_cube.shape[1] * data_cube.shape[2]
            x.sort()
            y = list(np.arange(0, data_cube.shape[1])) * data_cube.shape[2] 
            y.sort()
            y = y * data_cube.shape[0]
            z = list(np.arange(0, data_cube.shape[2]))
            z.sort()
            z = z * data_cube.shape[1] * data_cube.shape[0]
            c = data_cube[:, :, :]

            fig = plt.figure()
            ax = fig.add_subplot(projection='3d')
            scatter = ax.scatter(x,y,z,c=c)
            ax.set_xlabel('X Label')
            ax.set_ylabel('Y Label')
            ax.set_zlabel('Z Label')

            # produce a legend with the unique colors from the scatter
            legend1 = ax.legend(*scatter.legend_elements(),
                                loc='lower left', title="Values")
            ax.add_artist(legend1)

            plt.show()

    @staticmethod
    def visualize_3d_slices(hdf5_filepath, group_name="Test confocal alignment", dataset_name="dataset"):
                
        def plot_quadrants(ax, array, fixed_coord, cmap):
            """For a given 3d *array* plot a plane with *fixed_coord*, using four quadrants."""
            nx, ny, nz = array.shape
            index = {
                'x': (nx // 2, slice(None), slice(None)),
                'y': (slice(None), ny // 2, slice(None)),
                'z': (slice(None), slice(None), nz // 2),
            }[fixed_coord]
            plane_data = array[index]

            n0, n1 = plane_data.shape
            quadrants = [
                plane_data[:n0 // 2, :n1 // 2],
                plane_data[:n0 // 2, n1 // 2:],
                plane_data[n0 // 2:, :n1 // 2],
                plane_data[n0 // 2:, n1 // 2:]
            ]

            min_val = array.min()
            max_val = array.max()

            cmap = plt.get_cmap(cmap)

            for i, quadrant in enumerate(quadrants):
                facecolors = cmap((quadrant - min_val) / (max_val - min_val))
                if fixed_coord == 'x':
                    Y, Z = np.mgrid[0:ny // 2, 0:nz // 2]
                    X = nx // 2 * np.ones_like(Y)
                    Y_offset = (i // 2) * ny // 2
                    Z_offset = (i % 2) * nz // 2
                    ax.plot_surface(X, Y + Y_offset, Z + Z_offset, rstride=1, cstride=1,
                                    facecolors=facecolors, shade=False)
                elif fixed_coord == 'y':
                    X, Z = np.mgrid[0:nx // 2, 0:nz // 2]
                    Y = ny // 2 * np.ones_like(X)
                    X_offset = (i // 2) * nx // 2
                    Z_offset = (i % 2) * nz // 2
                    ax.plot_surface(X + X_offset, Y, Z + Z_offset, rstride=1, cstride=1,
                                    facecolors=facecolors, shade=False)
                elif fixed_coord == 'z':
                    X, Y = np.mgrid[0:nx // 2, 0:ny // 2]
                    Z = nz // 2 * np.ones_like(X)
                    X_offset = (i // 2) * nx // 2
                    Y_offset = (i % 2) * ny // 2
                    ax.plot_surface(X + X_offset, Y + Y_offset, Z, rstride=1, cstride=1,
                                    facecolors=facecolors, shade=False)

        def figure_3D_array_slices(array, cmap=None):
            """Plot a 3d array using three intersecting centered planes."""
            fig = plt.figure()
            ax = fig.add_subplot(projection='3d')
            ax.set_box_aspect(array.shape)
            plot_quadrants(ax, array, 'x', cmap=cmap)
            plot_quadrants(ax, array, 'y', cmap=cmap)
            plot_quadrants(ax, array, 'z', cmap=cmap)
            return fig, ax

        with h5py.File(hdf5_filepath, 'r') as hdf5_file:
            data_group = hdf5_file.get(group_name)
            # data_cube = data_group.require_dataset(dataset_name, shape=dataset_shape, dtype=np.int32, exact=False)
            data_cube = data_group.get(dataset_name)
            if data_cube is None :
                raise ValueError(f"Could not find dataset {dataset_name} in group {group_name} inside hdf5file : {hdf5_filepath}")
            
            r_square = np.array(data_cube)
            figure_3D_array_slices(r_square, cmap='coolwarm')
        
        plt.show()

    def convert_calib_type(hdf5_filepath, group_name="XRF_analysis"):
        with h5py.File(hdf5_filepath, 'r') as initial_file : 
            dset = initial_file[f'{group_name}/data']
            with h5py.File(hdf5_filepath+'_new', 'w') as new_file :
                new_file.create_dataset('data', dset.shape, data=dset)

def test_visu_3d():

    hdf5_filepath = r'C:\Users\CXRF\Code\depthpaint-c-xrf-interface\corapp\tabs\fonts\icons\test_temp.hdf5'
    Hdf5Handler.visualize_3d_slices(hdf5_filepath)

def test_convert_calib():

    hdf5_filepath = r"C:\Users\CXRF\Code\depthpaint-c-xrf-interface\corapp\tests\results\cobayeOne\MA-XRF\test_cobayeOne_140x140_250x250_50ms.hdf5"
    Hdf5Handler.convert_calib_type(hdf5_filepath)

if __name__ == "__main__":

    test_visu_3d()
    # test_convert_calib()
