import { Component, OnDestroy, OnInit, Input, Output, EventEmitter, ChangeDetectionStrategy } from '@angular/core';
import { Subject, Observable } from 'rxjs';

import { faCoffee, faPenToSquare, faTrash, faTimes, faFloppyDisk } from '@fortawesome/free-solid-svg-icons';

@Component({
  selector: 'bob-mask-controls',
  templateUrl: './mask-controls.component.html',
  styleUrls: ['./mask-controls.component.scss'],
  changeDetection: ChangeDetectionStrategy.OnPush
})
export class MaskControlsComponent implements OnInit, OnDestroy {

  private _ngUnsubscribe$: Subject<void> = new Subject<void>();  

  @Input() Heading: string = "Privacy Mask Controls";
  @Input() HeadingEdit: string = "Editing Privacy Mask";
  @Input() SubHeading: string = "";
  @Input() SubHeadingEdit: string = "Draw the areas you want to exclude";

  @Output() EditMode = new EventEmitter<boolean>();
  
  @Output() Edit = new EventEmitter();
  @Output() Cancel = new EventEmitter();
  @Output() Delete = new EventEmitter();
  @Output() Save = new EventEmitter();
  @Output() Clear = new EventEmitter();

  faCoffee = faCoffee;
  faPenToSquare = faPenToSquare;
  faTrash = faTrash;
  faTimes = faTimes;
  faFloppyDisk = faFloppyDisk;

  editMode: boolean = false;

  constructor() {    
  }

  ngOnInit(): void {
  }

  edit(): void {
    this.Edit.emit()
    this.editMode = !this.editMode;
    this.EditMode.emit(this.editMode)    
  }

  cancel(): void {
    this.Cancel.emit()
    this.editMode = !this.editMode;
    this.EditMode.emit(this.editMode)
  }

  delete(): void {
    this.Delete.emit()
  }
  
  save(): void {
    this.Save.emit()
  }
  
  clear(): void {
    this.Clear.emit()
  }

  ngOnDestroy(): void {
    this._ngUnsubscribe$.next();
    this._ngUnsubscribe$.complete();
  }
}
