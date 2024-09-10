import { HttpClient, HttpHeaders } from "@angular/common/http";
import { Injectable } from "@angular/core";
import { catchError, map, Observable } from "rxjs";

import { ApiServiceBase } from '../../../core/services/api-service-base.service'
import { CameraQuery, CameraDto } from '../models';

@Injectable({
  providedIn: 'root',
})
export class VisionService extends ApiServiceBase {

  constructor(private http: HttpClient) {
    super(http);
  }

  cameraDetails(input: CameraQuery): Observable<CameraDto> {
    return super.getRequest<CameraDto>(
        `${this.baseUrl}/api/vision?CameraName=${encodeURI(input.cameraName)}`);
  }
}