# README #

## CI/CD ##

| service   |  Master                                                                                                                                                                                                                                                                  |
|:----------|:------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------:|
| Travis    | [![Build Status](https://travis-ci.org/CNR-STIIMA-IRAS/eigen_control_toolbox.svg?branch=master)](https://travis-ci.org/CNR-STIIMA-IRAS/eigen_control_toolbox)                                                                                                                                  |
| Codecov   | [![codecov](https://codecov.io/gh/CNR-STIIMA-IRAS/eigen_control_toolbox/branch/master/graph/badge.svg)](https://codecov.io/gh/CNR-STIIMA-IRAS/eigen_control_toolbox)                                                                                                                           | 
| Codacy    | [![Codacy Badge](https://api.codacy.com/project/badge/Grade/7f1834c02aa84b959ee9b7529deb48d6)](https://app.codacy.com/gh/CNR-STIIMA-IRAS/eigen_control_toolbox?utm_source=github.com&utm_medium=referral&utm_content=CNR-STIIMA-IRAS/eigen_control_toolbox&utm_campaign=Badge_Grade_Dashboard) | 
| Fossa     | [![FOSSA Status](https://app.fossa.com/api/projects/git%2Bgithub.com%2FCNR-STIIMA-IRAS%2Feigen_control_toolbox.svg?type=shield)](https://app.fossa.com/projects/git%2Bgithub.com%2FCNR-STIIMA-IRAS%2Feigen_control_toolbox?ref=badge_shield)                                                   |

## Aim ##

The package has been designed to 

## Usage ##

### Dependencies ###



### Parameters ###

```yaml
  ~/appenders: ['file', 'screen']                 # Mandatory
                                                  # A vector od dimension 1 or 2, where you can select if the output will be streamed to file, to screen or to both
                                                  # (the order in the vector is not important)

  ~/levels: [ debug|info|warn|error|fatal, ..]    # Optional
                                                  # A vector where you can select the verbosity level for each appender.
                                                  # If not present, or if the size of the vector is different from the dimension of the appenders,
                                                  # the default value is superimposed.
                                                  # Default: 'debug' for all the appenders

  ~/pattern_layout: '...'                         # Optional
                                                  # look at https://svn.apache.org/repos/asf/logging/site/trunk/docs/log4cxx/apidocs/classlog4cxx_1_1_pattern_layout.html"
                                                  # This allows you to define the log pattern.
                                                  # Default is: [%5p] [%d{HH:mm:ss,SSS}][%r][%M:%L]: %m%n

  ~/file_name: 'file_name'                        # Optional
                                                  # If 'file' is selected, this is the path of the log file.
                                                  # If any absolute path is indicated it saves under the default location.
                                                  # Default: ~/.ros/log/[logger_id].log

  ~/append_date_to_file_name: true|false          # Optional
                                                  # The named file will be appended with the YYMMDD-HH:MM::SS of creation
                                                  # Default: false

  ~/append_to_file: true|false                    # Optional
                                                  # If true, the content is appended to file. The new content starts with a clear header (with data and start time).
                                                  # If not, the log file is overwritten.
                                                  # Default: true
```

### Class initialization and usage ###

There are two constructors:

```cpp
TraceLogger( const std::string& logger_id )
TraceLogger( const std::string& logger_id, const std::string& param_namespace, const bool star_header )
```

The first does not initialize the instance of the class, and the function `init()` must be called afterwards.
The second initializes the instance of the class.

If the initialization failed, the class superimpose default values unless the user explicitly indicates to not use the default values.

#### Example of usage ####

```cpp
  
```

#### Utilities with the package ####

The ANSI Colors are defined as an inline function

```cpp

```

The macros to be used within the code are:

```cpp

#define CNR_RETURN_NOTOK_THROTTLE( logger, var, period, ...)
```

### Contribution guidelines ###

### Contact ###

<mailto:<mailto:manuel.beschi@stiima.cnr.it>>
<mailto:<mailto:nicola.pedrocchi@stiima.cnr.it>>

## License ##
[![FOSSA Status](https://app.fossa.com/api/projects/git%2Bgithub.com%2FCNR-STIIMA-IRAS%2Feigen_control_toolbox.svg?type=large)](https://app.fossa.com/projects/git%2Bgithub.com%2FCNR-STIIMA-IRAS%2Feigen_control_toolbox?ref=badge_large)