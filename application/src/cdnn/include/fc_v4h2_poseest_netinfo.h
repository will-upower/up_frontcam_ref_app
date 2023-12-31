{
    .cl_cnn0_filename = "./fc_v4h2/poseest/CNN0.bin",
    .cl_dma0_filename = "./fc_v4h2/poseest/SDMAC0.bin",
    .cl_dma1_filename = "./fc_v4h2/poseest/SDMAC1.bin",
    .cl_dma2_filename = "./fc_v4h2/poseest/SDMAC2.bin",
    .cl_dma3_filename = "./fc_v4h2/poseest/SDMAC3.bin",
    .weight_filename  = "./fc_v4h2/poseest/weight.bin",
    .orig_weight_addr = 0xc8000000,
    .orig_output_addr = 0xdc100000,
    .output_size      = 0x0000b230,
    .output_num       = 2,
    .qdata_filename   = "",
    .usecore          = IMPFWDEMO_CNN_ONLY,
    .output_fp32      = 0,
    .target_dsp_core  = 0,
    .out_cf =
    {
        {
            .output_name  = "./fc_v4h2/poseest/output1_t.bin",
            .offset       = 0x00000000,
            .ch           = 38,
            .ch_stride    = 784,
            .width        = 28,
            .height       = 28,
            .bytepp       = 1,
            .scale_factor = 1.000000,
            .zero_point   = 0,
        },
            {
            .output_name  = "./fc_v4h2/poseest/output2_t.bin",
            .offset       = 0x00007800,
            .ch           = 19,
            .ch_stride    = 784,
            .width        = 28,
            .height       = 28,
            .bytepp       = 1,
            .scale_factor = 1.000000,
            .zero_point   = 0,
        }
        
    },
},