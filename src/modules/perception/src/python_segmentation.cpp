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

    ~Impl() {
        Py_XDECREF(pProcessMethod);
        Py_XDECREF(pProcessor);
    }
};

PythonSegmentation::PythonSegmentation(const std::string& config_path, const std::string& weight_path)
    : pImpl(std::make_unique<Impl>()) {

    // Initialize Python interpreter
    Py_Initialize();
    if (!Py_IsInitialized()) {
        throw std::runtime_error("Failed to initialize Python interpreter");
    }

    // Initialize NumPy
    if (_import_array() < 0) {
        PyErr_Print();
        throw std::runtime_error("Failed to import numpy.core.multiarray");
    }

    try {
        // Get package path
        std::string pkg_path = ros::package::getPath("perception");

        // Set Python paths
        std::string python_path_cmd =
            "import sys\n"
            "sys.path.insert(0, '" + pkg_path + "/scripts')\n"
            "sys.path.insert(0, '" + pkg_path + "/python')\n"
            "sys.path.insert(0, '" + config_path.substr(0, config_path.find_last_of("/")) + "')\n";

        PyRun_SimpleString(python_path_cmd.c_str());

        // Import module
        PyObject* pModule = PyImport_ImportModule("perception.segmentation");
        if (!pModule) {
            PyErr_Print();
            throw std::runtime_error("Failed to import Python module 'perception.segmentation'");
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
        if (!pArgs) {
            Py_XDECREF(pClass);
            throw std::runtime_error("Failed to create arguments tuple");
        }

        PyTuple_SetItem(pArgs, 0, PyUnicode_FromString(config_path.c_str()));
        PyTuple_SetItem(pArgs, 1, PyUnicode_FromString(weight_path.c_str()));

        pImpl->pProcessor = PyObject_CallObject(pClass, pArgs);
        Py_DECREF(pArgs);
        Py_DECREF(pClass);

        if (!pImpl->pProcessor) {
            PyErr_Print();
            throw std::runtime_error("Failed to create SegmentationProcessor instance");
        }

        // Get process method
        pImpl->pProcessMethod = PyObject_GetAttrString(pImpl->pProcessor, "process");
        if (!pImpl->pProcessMethod || !PyCallable_Check(pImpl->pProcessMethod)) {
            PyErr_Print();
            throw std::runtime_error("Failed to get process method");
        }

    } catch (...) {
        Py_Finalize();
        throw;
    }
}

PythonSegmentation::~PythonSegmentation() {
    // Impl's destructor handles cleanup
}

cv::Mat PythonSegmentation::process(const cv::Mat& input) {
    if (!pImpl->pProcessMethod) {
        throw std::runtime_error("Processor not initialized");
    }

    // Convert cv::Mat to numpy array
    npy_intp dimensions[3] = {input.rows, input.cols, input.channels()};
    PyObject* pInputArray = PyArray_SimpleNewFromData(
        3, dimensions, NPY_UINT8, (void*)input.data
    );
    if (!pInputArray) {
        PyErr_Print();
        throw std::runtime_error("Failed to create numpy array from cv::Mat");
    }

    PyObject* pResult = nullptr;
    cv::Mat result;

    try {
        // Call Python method
        pResult = PyObject_CallFunctionObjArgs(pImpl->pProcessMethod, pInputArray, nullptr);
        Py_DECREF(pInputArray);

        if (!pResult) {
            PyErr_Print();
            throw std::runtime_error("Python process method failed");
        }

        // Convert result to cv::Mat
        PyArrayObject* np_array = reinterpret_cast<PyArrayObject*>(pResult);
        if (!np_array || PyArray_NDIM(np_array) != 3 || PyArray_TYPE(np_array) != NPY_UINT8) {
            throw std::runtime_error("Python returned invalid array format");
        }

        result = cv::Mat(
            static_cast<int>(PyArray_DIM(np_array, 0)),
            static_cast<int>(PyArray_DIM(np_array, 1)),
            CV_8UC3,
            PyArray_DATA(np_array)
        ).clone();

        Py_DECREF(pResult);

    } catch (...) {
        Py_XDECREF(pInputArray);
        Py_XDECREF(pResult);
        throw;
    }

    return result;
}