#!/usr/bin/env python
import numpy as np

class SimpleKalmanFilter():
    # mea_e: Measurement uncertainty
    # est_e: Estimation uncertainty
    # q:     Process variance (usually 0.01)
    def __init__(self,mea_e,est_e,q):
        self.mea_e = mea_e
        self.est_e = est_e
        self.q = q
        self.kf_gain = 0
        self.current_est = 0
        self.last_est = 0

    def predict(self, mea):
        self.kf_gain = self.est_e/(self.est_e + self.mea_e)
        self.current_est = self.last_est + self.kf_gain*(mea-self.last_est)
        self.est_e = (1.0 - self.kf_gain)*self.est_e + abs(self.last_est-self.current_est)*self.q
        self.last_est = self.current_est

        return self.current_est
