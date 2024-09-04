import { ComponentFixture, TestBed } from '@angular/core/testing';

import { AppRootComponent } from './app-root.component';

describe('AppRootComponent', () => {
  let component: AppRootComponent;
  let fixture: ComponentFixture<AppRootComponent>;

  beforeEach(async () => {
    await TestBed.configureTestingModule({
      declarations: [ AppRootComponent ]
    })
    .compileComponents();

    fixture = TestBed.createComponent(AppRootComponent);
    component = fixture.componentInstance;
    fixture.detectChanges();
  });

  it('should create', () => {
    expect(component).toBeTruthy();
  });
});
