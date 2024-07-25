// This file can be replaced during build by using the `fileReplacements` array.
// `ng build` replaces `environment.ts` with `environment.prod.ts`.
// The list of file replacements can be found in `angular.json`.

export const environment = {
  appName: require('../../package.json').name,
  production: true,
  version: require('../../package.json').version + '-prod',
  settings: {
    autoLogoutInterval: 60,
    appApi: {
      baseUrl: 'http://localhost:5000/api'
    },
    serices: {
      notification: {
        displayDuration: {
          info: 5000,
          warn: 7500,
          error: 10000
        }
      }
    },      
    ui: {
      titleEllipsis: 30,
      local: 'en-GB',
      dateLocal: 'en-GB',      
      dialogWidth: '1050px',
      pipeFormat : {
        currency: 'GBP',
        date: 'dd/MM/yyyy HH:mm:ss'
      }
    }
  }
};

/*
 * For easier debugging in development mode, you can import the following file
 * to ignore zone related error stack frames such as `zone.run`, `zoneDelegate.invokeTask`.
 *
 * This import should be commented out in production mode because it will have a negative impact
 * on performance if an error is thrown.
 */
// import 'zone.js/plugins/zone-error';  // Included with Angular CLI.
