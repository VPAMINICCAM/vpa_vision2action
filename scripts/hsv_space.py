#!/usr/bin/env python3
# -*- coding: utf-8 -*-

class HSVSpace:
    
    def __init__(self,h_u,h_l,s_u,s_l,v_u,v_l) -> None:
        self._h_upper = h_u
        self._h_lower = h_l
        self._s_upper = s_u
        self._s_lower = s_l
        self._v_upper = v_u
        self._v_lower = v_l
    
    def _generate_lower_mask(self):
        return np.array([self._h_lower,self._s_lower,self._v_lower])
    
    def _generate_upper_mask(self):
        return np.array([self._h_upper,self._s_upper,self._v_upper])
    
    def generate_mask(self,hsv_image):
        _mask   = cv2.inRange(hsv_image,self._generate_lower_mask(),self._generate_upper_mask())
        _kernel = np.ones((9,9),np.uint8)
        _mask   = cv2.morphologyEx(_mask,cv2.MORPH_CLOSE,_kernel)
        return _mask