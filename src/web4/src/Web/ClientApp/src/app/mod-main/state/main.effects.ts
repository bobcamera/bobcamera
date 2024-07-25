import { Injectable } from '@angular/core';
import { HttpErrorResponse, HttpResponse } from '@angular/common/http';

import { ErrorService } from '../services';

@Injectable()
export class MainEffects {

  // https://blog.angular-university.io/debug-rxjs/

  constructor(private errorService: ErrorService) {
  }

  protected processError(error: HttpErrorResponse, notify: boolean = true): void {

    console.error(`Error during server communications. Code: ${error.status}. Error ${error.message}`);    

    switch (error.status) {
        case 401:
        case 403:
          //this._routerService.toAccessDenied();
        break;
        default:
          if (notify) {
            this.errorService.HandleError('Error during server communications', error);
          }
          break;
    }
  } 
}