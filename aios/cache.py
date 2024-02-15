# import json
# from trendlines import returnFunctions

# launch_coeffs, angle_coeffs = returnFunctions()

# coeffs = {
#     "launch": {
#         "a": launch_coeffs[0],
#         "b": launch_coeffs[1],
#         "c": launch_coeffs[2]
#     },
#     "angle": {
#         "a": angle_coeffs[0],
#         "b": angle_coeffs[1],
#         "c": angle_coeffs[2]
#     }
# }

coeffs = {
    "launch": {
        "a": 0.001062497903615323,
        "b": 0.7471887693867607,
        "c": 5.478684772893152
    },
    "angle": {
        "a": 7.752605287137327e-05,
        "b": -0.007477191156019437,
        "c": 1.0663329616679225
    }
}

print(coeffs)