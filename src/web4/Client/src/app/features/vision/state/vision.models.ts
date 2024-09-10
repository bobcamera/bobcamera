import { HttpErrorResponse } from '@angular/common/http';
import { CameraDto, AppInfoDto, ImageStreamTypeEnum } from '../models';

export interface VisionState {
    heading: string;
    menuPanelExpanded: boolean;
    contextPanelExpanded: boolean;
    imageStreamType: ImageStreamTypeEnum;
    displayPrivacyMaskControls: boolean;
    displayDetectionMaskControls: boolean;
    displayAppState: boolean;
  
    message: string;
  
    error: HttpErrorResponse;
    loading: boolean;
    loadingMessage: string;
  
    enableCameraPolling: boolean;
    camera: CameraDto;
  
    bobInfo: AppInfoDto;
  
    maskEditMode: boolean;
    maskSvg: string;
  }