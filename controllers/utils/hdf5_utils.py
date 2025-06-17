# -- coding: utf-8 -*-

# builtin imports
import threading
from datetime import datetime

# Third-party imports
import matplotlib.pyplot as plt
import h5py
import numpy as np

# Static Class
class Hdf5Handler():

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

def main():
    # Example usage of the Hdf5Handler class
    save_filepath = "example_data.hdf5"
    output_data = np.random.rand(100, 100, 512)  # Simulated XRF data
    calibration = np.random.rand(3)
    mono_xrf_data = output_data.sum(axis=(0, 1))  # Simulated mono XRF data
    calibration_mono_xrf = np.random.rand(3)
    picture = np.random.rand(256, 256, 3) * 255  # Simulated RGB image
    metadata = {
        "project_name": "Example Project",
        "object_name": "Example Object",
        "user": "Example User",
        "date": datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    }

    Hdf5Handler.save_data_to_hdf5(
        save_filepath,
        output_data,
        calibration,
        project_name="Example Project",
        object_name="Example Object",
        analysis_name="XRF Analysis",
        dataset_name="2D-XRF Mapping",
        mono_xrf_dataset_name="XRF point",
        mono_xrf_data=mono_xrf_data,
        calibration_mono_xrf=calibration_mono_xrf,
        picture=picture,
        metadata=metadata
    )

if __name__ == "__main__":

    main()
