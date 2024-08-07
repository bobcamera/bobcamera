import { DEFAULT_CURRENCY_CODE, LOCALE_ID, NgModule } from '@angular/core';
import { HttpClientModule } from '@angular/common/http';
import { FormsModule, ReactiveFormsModule } from '@angular/forms';

import { MatAutocompleteModule } from '@angular/material/autocomplete';
import { MatBadgeModule } from '@angular/material/badge';
import { MatButtonModule } from '@angular/material/button';
import { MatButtonToggleModule } from '@angular/material/button-toggle';
import { MatCardModule } from '@angular/material/card';
import { MatCheckboxModule } from '@angular/material/checkbox';
import { MatChipsModule } from '@angular/material/chips';
import { MatDatepickerModule } from '@angular/material/datepicker';
import { MatDialogModule, MAT_DIALOG_DEFAULT_OPTIONS } from '@angular/material/dialog';
import { MatDividerModule } from '@angular/material/divider';
import { MatExpansionModule } from '@angular/material/expansion';
import { MatGridListModule } from '@angular/material/grid-list';
import { MatIconModule } from '@angular/material/icon';
import { MatInputModule } from '@angular/material/input';
import { MatListModule } from '@angular/material/list';
import { MatMenuModule } from '@angular/material/menu';
import { MatNativeDateModule } from '@angular/material/core';
import { MatProgressBarModule } from '@angular/material/progress-bar';
import { MatProgressSpinnerModule } from '@angular/material/progress-spinner';
import { MatRadioModule } from '@angular/material/radio';
import { MatRippleModule } from '@angular/material/core';
import { MatSelectModule } from '@angular/material/select';
import { MatSidenavModule } from '@angular/material/sidenav';
import { MatSliderModule } from '@angular/material/slider';
import { MatSlideToggleModule } from '@angular/material/slide-toggle';
import { MatSnackBarModule } from '@angular/material/snack-bar';
import { MatStepperModule } from '@angular/material/stepper';
import { MatTableModule } from '@angular/material/table';
import { MatTabsModule } from '@angular/material/tabs';
import { MatToolbarModule } from '@angular/material/toolbar';
import { MatTooltipModule } from '@angular/material/tooltip';
import { MatTreeModule } from '@angular/material/tree';
import { MatPaginatorModule } from '@angular/material/paginator';
import { MatSortModule } from '@angular/material/sort';
import { MAT_DATE_LOCALE,  } from '@angular/material/core';
import { MatFormFieldModule, MAT_FORM_FIELD_DEFAULT_OPTIONS } from '@angular/material/form-field';

import { environment } from 'src/environments/environment';

import { DATE_PIPE_DEFAULT_OPTIONS } from '@angular/common';

import { FontAwesomeModule } from '@fortawesome/angular-fontawesome';

/*import { MTX_DATETIME_FORMATS } from '@ng-matero/extensions/core';
import { MtxDatetimepickerModule, } from '@ng-matero/extensions/datetimepicker';
import { MtxLuxonDatetimeModule } from '@ng-matero/extensions-luxon-adapter';
import { MtxLoaderModule } from '@ng-matero/extensions/loader';
import { MtxSplitModule } from '@ng-matero/extensions/split';*/

const ANGULAR_MODULES: any[] = [
  FormsModule, ReactiveFormsModule, HttpClientModule,
];

const MATERIAL_MODULES: any[] = [MatAutocompleteModule, MatBadgeModule, MatButtonModule, MatButtonToggleModule, MatCardModule,
  MatCheckboxModule, MatChipsModule, MatDatepickerModule, MatDialogModule, MatDividerModule, MatExpansionModule, MatGridListModule,
  MatIconModule, MatInputModule, MatListModule, MatMenuModule, MatNativeDateModule, MatProgressBarModule, MatProgressSpinnerModule,
  MatRadioModule, MatRippleModule, MatSelectModule, MatSidenavModule, MatSliderModule, MatSlideToggleModule, MatSnackBarModule,
  MatStepperModule, MatTableModule, MatTabsModule, MatToolbarModule, MatTooltipModule, MatTreeModule, MatPaginatorModule, MatSortModule,
  MatFormFieldModule];

const OTHER_UI_MODULES: any[] = [FontAwesomeModule];//MtxLuxonDatetimeModule , MtxDatetimepickerModule, MtxLoaderModule, MtxSplitModule];

@NgModule({
  imports: [
    ANGULAR_MODULES,
    MATERIAL_MODULES,
    OTHER_UI_MODULES,
  ],
  declarations: [

  ],
  exports: [
    ANGULAR_MODULES,
    MATERIAL_MODULES,
    OTHER_UI_MODULES,
  ],
  providers: [

    // Locale providers
    { provide: LOCALE_ID, useValue: environment.settings.ui.local },
    { provide: MAT_DATE_LOCALE, useValue: environment.settings.ui.dateLocal }, 
    
    // Styling option providers
    { provide: MAT_DIALOG_DEFAULT_OPTIONS, useValue: { width: environment.settings.ui.dialogWidth } },
    { provide: MAT_FORM_FIELD_DEFAULT_OPTIONS, useValue: { appearance: 'outline', floatLabel: 'always' } },

    // Default pipe formatters
    { provide: DEFAULT_CURRENCY_CODE, useValue: environment.settings.ui.pipeFormat.currency },    
    { provide: DATE_PIPE_DEFAULT_OPTIONS, useValue: { dateFormat: environment.settings.ui.pipeFormat.date } },
    
    // For the formatting tokens used in string parsing and formatting with Luxon see at https://github.com/moment/luxon/blob/master/docs/parsing.md and https://github.com/moment/luxon/blob/master/docs/formatting.md  
    /*{ provide: MTX_DATETIME_FORMATS,
      useValue: {
        parse: {
          dateInput: 'dd/LL/yyyy',
          monthInput: 'LLLL',
          yearInput: 'yyyy',
          datetimeInput: 'dd/LL/yyyy HH:mm',
          timeInput: 'HH:mm',
        },
        display: {
          dateInput: 'dd/LL/yyyy',                  
          monthInput: 'LLLL',                      
          yearInput: 'yyyy',                        
          datetimeInput: 'dd/LL/yyyy HH:mm',                                             
          timeInput: 'HH:mm:ss',
          monthYearLabel: 'yyyy',
          dateA11yLabel: 'DDD',
          monthYearA11yLabel: 'LLLL yyyy',
          popupHeaderDateLabel: 'dd LLL, ccc',
        },
      },
    },*/
  ],
})
export class UiModule { }