#include "python_segmentation.h"
#include <ros/package.h>
#include <Python.h>
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <numpy/arrayobject.h>
#include <stdexcept>
#include <iostream>

struct PythonSegmentation::Impl {
    PyObject* pProcessor = nullptr;
    PyObject* pProcessMethod = nullptr;
    bool initialized = false;

    ~Impl() {
        PyGILState_STATE gstate = PyGILState_Ensure();
        Py_XDECREF(pProcessMethod);
        Py_XDECREF(pProcessor);
        PyGILState_Release(gstate);
    }
};

PythonSegmentation::PythonSegmentation(const std::string& config_path, const std::string& weight_path)
    : pImpl(std::make_unique<Impl>()) {

    // Initialize Python interpreter
    Py_InitializeEx(0);
    if (!Py_IsInitialized()) {
        throw std::runtime_error("Python interpreter initialization failed");
    }

    // Initialize thread support
    if (!PyEval_ThreadsInitialized()) {
        PyEval_InitThreads();
    }
    PyGILState_STATE gstate = PyGILState_Ensure();

    try {
        // MANUAL NUMPY INITIALIZATION (FIXED VERSION)
        if (_import_array() < 0) {
            PyErr_Print();
            throw std::runtime_error("NumPy initialization failed");
        }

        // Get package path
        std::string pkg_path = ros::package::getPath("perception");

        // Set Python paths
        std::string python_path_cmd =
            "import sys\n"
            "sys.path.insert(0, '" + pkg_path + "/scripts')\n"
            "sys.path.insert(0, '" + pkg_path + "/lib')\n"  // Explicitly add lib
            "sys.path.insert(0, '" + pkg_path + "/config')\n";

        PyRun_SimpleString(python_path_cmd.c_str());

        // Import module
        PyObject* pModule = PyImport_ImportModule("segmentation");
        if (!pModule) {
            PyErr_Print();
            throw std::runtime_error("Failed to import segmentation module");
        }

        // Get processor class
        PyObject* pClass = PyObject_GetAttrString(pModule, "SegmentationProcessor");
        Py_DECREF(pModule);
        if (!pClass || !PyCallable_Check(pClass)) {
            Py_XDECREF(pClass);
            PyErr_Print();
            throw std::runtime_error("Failed to get SegmentationProcessor class");
        }

        // Create processor instance
        PyObject* pArgs = PyTuple_New(2);
        PyTuple_SetItem(pArgs, 0, PyUnicode_FromString(config_path.c_str()));
        PyTuple_SetItem(pArgs, 1, PyUnicode_FromString(weight_path.c_str()));

        pImpl->pProcessor = PyObject_CallObject(pClass, pArgs);
        Py_DECREF(pArgs);
        Py_DECREF(pClass);

        if (!pImpl->pProcessor) {
            PyErr_Print();
            throw std::runtime_error("Failed to create processor instance");
        }

        // Get process method
        pImpl->pProcessMethod = PyObject_GetAttrString(pImpl->pProcessor, "process");
        if (!pImpl->pProcessMethod || !PyCallable_Check(pImpl->pProcessMethod)) {
            PyErr_Print();
            throw std::runtime_error("Failed to get process method");
        }

        pImpl->initialized = true;
        PyGILState_Release(gstate);
    } catch (...) {
        PyGILState_Release(gstate);
        Py_Finalize();
        throw;
    }
}

PythonSegmentation::~PythonSegmentation() {
    // Impl destructor handles cleanup
}

cv::Mat PythonSegmentation::process(const cv::Mat& input) {
    if (!pImpl->initialized) {
        throw std::runtime_error("Processor not initialized");
    }

    PyGILState_STATE gstate = PyGILState_Ensure();
    PyObject* pInputArray = nullptr;
    PyObject* pResult = nullptr;
    cv::Mat result;

    try {
        // Convert cv::Mat to numpy array
        npy_intp dimensions[3] = {input.rows, input.cols, input.channels()};
        pInputArray = PyArray_SimpleNewFromData(3, dimensions, NPY_UINT8, (void*)input.data);
        if (!pInputArray) {
            PyErr_Print();
            throw std::runtime_error("Failed to create numpy array");
        }

        // Call Python method
        pResult = PyObject_CallFunctionObjArgs(pImpl->pProcessMethod, pInputArray, nullptr);
        Py_DECREF(pInputArray);

        if (!pResult) {
            PyErr_Print();
            throw std::runtime_error("Python processing failed");
        }

        // Convert result to cv::Mat
        PyArrayObject* np_array = reinterpret_cast<PyArrayObject*>(pResult);
        if (PyArray_NDIM(np_array) != 3 || PyArray_TYPE(np_array) != NPY_UINT8) {
            throw std::runtime_error("Invalid numpy array format");
        }

        result = cv::Mat(
            static_cast<int>(PyArray_DIM(np_array, 0)),
            static_cast<int>(PyArray_DIM(np_array, 1)),
            CV_8UC3,
            PyArray_DATA(np_array)
        ).clone();

        Py_DECREF(pResult);
        PyGILState_Release(gstate);
    } catch (...) {
        Py_XDECREF(pInputArray);
        Py_XDECREF(pResult);
        PyGILState_Release(gstate);
        throw;
    }

    return result;
}