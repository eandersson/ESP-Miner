idf_component_register(SRC_DIRS "."
                       INCLUDE_DIRS "." "../../stratum/test/stubs"
                                    "../../../main/tasks" "../../../main/self_test"
                                    "../../../main/power"
                       REQUIRES cmock stratum stratum_v2 asic)
