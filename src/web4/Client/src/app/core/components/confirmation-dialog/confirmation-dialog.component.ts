import { Component, OnInit, Inject, ChangeDetectionStrategy, OnDestroy } from '@angular/core';
import { MAT_DIALOG_DATA, MatDialogRef } from '@angular/material/dialog';
import { Subject } from 'rxjs';

@Component({
  selector: 'bob-confirmation-dialog',
  templateUrl: './confirmation-dialog.component.html',
  styleUrls: ['./confirmation-dialog.component.scss'],
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class ConfirmationDialogComponent implements OnInit, OnDestroy {

  _ngUnsubscribe$: Subject<void> = new Subject<void>();

  _title: string = 'Confirmation Title << **GOES HERE**';
  _message: string = 'Confirmation message << **GOES HERE**';
  _cancelButton: string = 'Cancel';
  _acceptButton: string = 'Accept';

  constructor(private _dialogRef: MatDialogRef<ConfirmationDialogComponent>, @Inject(MAT_DIALOG_DATA) public data: any) {
    this._title = this.checkString(data?.title, this._title);
    this._message = this.checkString(data?.message, this._message);
    this._cancelButton = this.checkString(data?.cancelButton, this._cancelButton);
    this._acceptButton = this.checkString(data?.acceptButton, this._acceptButton);
  }

  ngOnInit() {
  }

  ngOnDestroy() {
    this._ngUnsubscribe$.next();
    this._ngUnsubscribe$.complete();
  }

  accept() {
    this._dialogRef.close(true);
  }

  cancel() {
    this._dialogRef.close(false);
  }

  checkString(input: string, d: string): string {
    if(typeof input!='undefined' && input){
      return input;
    }
    return d;
  }
}