#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "cape" for configuration "Release"
set_property(TARGET cape APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(cape PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "CXX"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "opencv_calib3d;opencv_core;opencv_dnn;opencv_features2d;opencv_flann;opencv_gapi;opencv_highgui;opencv_imgcodecs;opencv_imgproc;opencv_ml;opencv_objdetect;opencv_photo;opencv_stitching;opencv_video;opencv_videoio;opencv_alphamat;opencv_aruco;opencv_bgsegm;opencv_bioinspired;opencv_ccalib;opencv_cudaarithm;opencv_cudabgsegm;opencv_cudafeatures2d;opencv_cudafilters;opencv_cudaimgproc;opencv_cudalegacy;opencv_cudaobjdetect;opencv_cudaoptflow;opencv_cudastereo;opencv_cudawarping;opencv_cudev;opencv_datasets;opencv_dnn_objdetect;opencv_dnn_superres;opencv_dpm;opencv_face;opencv_freetype;opencv_fuzzy;opencv_hdf;opencv_hfs;opencv_img_hash;opencv_intensity_transform;opencv_line_descriptor;opencv_mcc;opencv_optflow;opencv_phase_unwrapping;opencv_plot;opencv_quality;opencv_rapid;opencv_reg;opencv_rgbd;opencv_saliency;opencv_sfm;opencv_shape;opencv_stereo;opencv_structured_light;opencv_superres;opencv_surface_matching;opencv_text;opencv_tracking;opencv_videostab;opencv_viz;opencv_wechat_qrcode;opencv_xfeatures2d;opencv_ximgproc;opencv_xobjdetect;opencv_xphoto"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libcape.a"
  )

list(APPEND _cmake_import_check_targets cape )
list(APPEND _cmake_import_check_files_for_cape "${_IMPORT_PREFIX}/lib/libcape.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
